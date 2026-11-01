#include "pynq_telemetry.h"
#include "zf_common_headfile.h"
#include <math.h>

typedef struct
{
    float speed_grid_s;
    float distance_grid;
    float x_grid;
    float y_grid;
    float heading_deg;
    uint32_t ticks;
    uint8_t input_flags;
} telemetry_sample_t;

static volatile telemetry_sample_t sample;
static uint32_t last_sent_tick;
static uint8_t sequence;
static volatile uint32_t raw_encoder[4], raw_gyro_sum, raw_t20, raw_t5;
void pynq_raw_encoder(const volatile int16_t *counts) {
    unsigned i;
    for(i=0;i<4;i++) raw_encoder[i] += (uint32_t)(int32_t)counts[i];
    raw_t20++;
}
void pynq_raw_gyro(int16_t z) {
    raw_gyro_sum += (uint32_t)(int32_t)z;
    raw_t5++;
}
static void put_u32(uint8_t *p, uint32_t v) {
    p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);p[2]=(uint8_t)(v>>16);p[3]=(uint8_t)(v>>24);
}

/* Called once per existing 20 ms odometry update. No UART access here. */
void pynq_telemetry_sample(float dx_grid, float dy_grid,
                         float x_grid, float y_grid, float heading_deg)
{
    float step;
    if (!isfinite(dx_grid) || !isfinite(dy_grid) ||
        !isfinite(x_grid) || !isfinite(y_grid) || !isfinite(heading_deg))
    {
        sample.input_flags = 0x80u;
        sample.ticks++;
        return;
    }
    step = sqrtf(dx_grid * dx_grid + dy_grid * dy_grid);
    if (!isfinite(step))
    {
        sample.input_flags = 0x80u;
        sample.ticks++;
        return;
    }
    sample.speed_grid_s = step / 0.020f;
    /* Accumulate raw encoder displacement; a pose reset is not travel. */
    sample.distance_grid += step;
    sample.x_grid = x_grid;
    sample.y_grid = y_grid;
    heading_deg = fmodf(heading_deg, 360.0f);
    if (heading_deg > 180.0f) heading_deg -= 360.0f;
    if (heading_deg < -180.0f) heading_deg += 360.0f;
    sample.heading_deg = heading_deg;
    sample.input_flags = 0;
    sample.ticks++;
}

static int32_t quantize(float value, int32_t low, int32_t high,
                        uint8_t flag, uint8_t *flags)
{
    int32_t rounded;
    float fraction;
    if (!isfinite(value))
    {
        *flags |= 0x80u;
        return 0;
    }
    if (value < (float)low) { *flags |= flag; return low; }
    if (value > (float)high) { *flags |= flag; return high; }
    rounded = (int32_t)value;
    fraction = value - (float)rounded;
    if (fraction >= 0.5f && rounded < high) rounded++;
    if (fraction <= -0.5f && rounded > low) rounded--;
    return rounded;
}

static void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static uint8_t crc8(const uint8_t *p, uint32_t length)
{
    uint8_t crc = 0;
    uint32_t i;
    for (i = 0; i < length; i++)
    {
        uint32_t bit;
        crc ^= p[i];
        for (bit = 0; bit < 8; bit++)
        {
            uint32_t shifted = (uint32_t)crc << 1;
            crc = (uint8_t)((crc & 0x80u) ? shifted ^ 0x07u : shifted);
        }
    }
    return crc;
}

/* Called in the normal main loop, never in the PIT ISR. */
void pynq_telemetry_poll(void)
{
    telemetry_sample_t current;
    uint32_t saved_mask;
    uint8_t frame[16], raw_frame[33];
    uint32_t raw_snapshot[7];
    unsigned raw_i;
    uint8_t flags;
    uint32_t distance;
    uint16_t speed;
    int16_t x, y, heading;

    if ((uint32_t)(sample.ticks - last_sent_tick) < 5u) return;
    saved_mask = __get_PRIMASK();
    __disable_irq();
    current.speed_grid_s = sample.speed_grid_s;
    current.distance_grid = sample.distance_grid;
    current.x_grid = sample.x_grid;
    current.y_grid = sample.y_grid;
    current.heading_deg = sample.heading_deg;
    current.ticks = sample.ticks;
    current.input_flags = sample.input_flags;
    for(raw_i=0;raw_i<4;raw_i++) raw_snapshot[raw_i]=raw_encoder[raw_i];
    raw_snapshot[4]=raw_gyro_sum;raw_snapshot[5]=raw_t20;raw_snapshot[6]=raw_t5;
    __set_PRIMASK(saved_mask);
    last_sent_tick = current.ticks;

    flags = current.input_flags;
    speed = (uint16_t)quantize(current.speed_grid_s * 1000.0f, 0, 65535, 1u, &flags);
    distance = (uint32_t)quantize(current.distance_grid * 1000.0f, 0, 16777215, 2u, &flags);
    x = (int16_t)quantize(current.x_grid * 1000.0f, -32768, 32767, 4u, &flags);
    y = (int16_t)quantize(current.y_grid * 1000.0f, -32768, 32767, 4u, &flags);
    heading = (int16_t)quantize(current.heading_deg * 100.0f, -18000, 18000, 8u, &flags);

    frame[0] = 0xa5u;
    frame[1] = 0x5au;
    frame[2] = sequence++;
    frame[3] = flags;
    put_u16(frame + 4, speed);
    frame[6] = (uint8_t)distance;
    frame[7] = (uint8_t)(distance >> 8);
    frame[8] = (uint8_t)(distance >> 16);
    put_u16(frame + 9, (uint16_t)x);
    put_u16(frame + 11, (uint16_t)y);
    put_u16(frame + 13, (uint16_t)heading);
    frame[15] = crc8(frame, 15);
    uart_write_buffer(UART_1, frame, sizeof(frame));
    raw_frame[0]=0xa6;raw_frame[1]=0x6a;raw_frame[2]=frame[2];raw_frame[3]=0;
    for(raw_i=0;raw_i<7;raw_i++) put_u32(raw_frame+4+4*raw_i,raw_snapshot[raw_i]);
    raw_frame[32]=crc8(raw_frame,32);
    uart_write_buffer(UART_1,raw_frame,sizeof(raw_frame));
}
