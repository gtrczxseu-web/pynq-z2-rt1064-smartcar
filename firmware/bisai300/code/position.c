#include "position.h"
#include "pynq_telemetry.h"
#include "control.h"    
#include "angle.h"   
#include "main.h"
#include <math.h>    

// 坐标单位：各全局轴自己的脉冲坐标（后续可转换为格子数）
// 南北方向1格使用 PULSES_PER_GRID_NS，东西方向1格使用 PULSES_PER_GRID_EW
//10991/94.5cm*20cm =2326
//按照轮子直径算约2463

//常用一个20cm=2326.0f


#define DEFAULT_MOVE_SPEED_PULSES (80.0f) // 目标移动速度上限（脉冲数/20ms）
//向北移动：
#define MOVE_POSITION_KP_NORTH_0_1    (0.071f)  // 0.1格及以下的位置P环增益
#define MOVE_POSITION_KP_NORTH_0_3    (0.0675f)  // 0.3格的位置P环增益
#define MOVE_POSITION_KP_NORTH_0_5    (0.061f)   // 0.5格的位置P环增益
#define MOVE_POSITION_KP_NORTH_1_0    (0.069f)   // 1格的位置P环增益
#define MOVE_POSITION_KP_NORTH_2_0    (0.091f)   // 2格的位置P环增益
#define MOVE_POSITION_KP_NORTH_2_5    (0.0995f)   // 2.5格的位置P环增益
#define MOVE_POSITION_KP_NORTH_3_0    (0.104f)   // 3格的位置P环增益
#define MOVE_POSITION_KP_NORTH_5_0    (0.105f)   // 5格的位置P环增益
#define MOVE_POSITION_KP_NORTH_7_0    (0.122f)   // 7格及以上的位置P环增益

//向南移动：
#define MOVE_POSITION_KP_SOUTH_0_1    (0.0798f)  // 0.1格及以下的位置P环增益
#define MOVE_POSITION_KP_SOUTH_0_3    (0.0675f)  // 0.3格的位置P环增益
#define MOVE_POSITION_KP_SOUTH_0_5    (0.0605f)   // 0.5格的位置P环增益
#define MOVE_POSITION_KP_SOUTH_1_0    (0.074f)   // 1格的位置P环增益
#define MOVE_POSITION_KP_SOUTH_2_0    (0.099f)   // 2格的位置P环增益
#define MOVE_POSITION_KP_SOUTH_2_5    (0.1095f)   // 2.5格的位置P环增益
#define MOVE_POSITION_KP_SOUTH_3_0    (0.115f)   // 3格的位置P环增益
#define MOVE_POSITION_KP_SOUTH_5_0    (0.140f)   // 5格的位置P环增益
#define MOVE_POSITION_KP_SOUTH_7_0    (0.139f)   // 7格及以上的位置P环增益

//向东移动
#define MOVE_POSITION_KP_EAST_0_1     (0.068f)  // 0.1格及以下的位置P环增益
#define MOVE_POSITION_KP_EAST_0_3     (0.074f)   // 0.3的位置P环增益
#define MOVE_POSITION_KP_EAST_0_5     (0.072f)   // 0.5格的位置P环增益
#define MOVE_POSITION_KP_EAST_1_0     (0.080f)   // 1格的位置P环增益
#define MOVE_POSITION_KP_EAST_2_0     (0.101f)   // 2格的位置P环增益
#define MOVE_POSITION_KP_EAST_2_5     (0.107f)   // 2.5格的位置P环增益
#define MOVE_POSITION_KP_EAST_3_0    (0.113f)   // 3格的位置P环增益
#define MOVE_POSITION_KP_EAST_5_0    (0.130f)   // 5格的位置P环增益
#define MOVE_POSITION_KP_EAST_7_0    (0.130f)   // 7格及以上的位置P环增益

//向西移动
#define MOVE_POSITION_KP_WEST_0_1     (0.0665f)  // 0.1格的位置P环增益
#define MOVE_POSITION_KP_WEST_0_3     (0.074f)   // 0.3格的位置P环增益
#define MOVE_POSITION_KP_WEST_0_5     (0.071f)   // 0.5格的位置P环增益
#define MOVE_POSITION_KP_WEST_1_0     (0.0788f)   // 1格的位置P环增益
#define MOVE_POSITION_KP_WEST_2_0     (0.0955f)   // 2格的位置P环增益
#define MOVE_POSITION_KP_WEST_2_5     (0.1025f)   // 2.5格的位置P环增益
#define MOVE_POSITION_KP_WEST_3_0     (0.109f)   // 3格的位置P环增益
#define MOVE_POSITION_KP_WEST_5_0     (0.1252f)   // 5格的位置P环增益
#define MOVE_POSITION_KP_WEST_7_0     (0.1242f)   // 7格及以上的位置P环增益

#define MOVE_STOP_SPEED_THRESHOLD (5.0f) // 判定车体平移已经稳定的速度阈值（脉冲数/20ms）
#define MOVE_SETTLE_CYCLES    (5)         // 平移速度连续低于稳定阈值的周期数（5*20ms=100ms）

// 全局坐标（脉冲单位）
static float g_pos_x = 0.0f;
static float g_pos_y = 0.0f;
static float g_logic_x = 0.0f;
static float g_logic_y = 0.0f;


// 定点移动控制变量
volatile uint8_t g_is_moving = 0;
static uint8_t g_move_held = 0;
static uint8_t g_current_direction;          // 当前移动方向
static float g_target_x = 0.0f;              // 目标X坐标（脉冲）
static float g_target_y = 0.0f;              // 目标Y坐标（脉冲）
static float g_logic_target_x = 0.0f;        // 理论目标X坐标（脉冲）
static float g_logic_target_y = 0.0f;        // 理论目标Y坐标（脉冲）
static uint8_t g_move_settle_count = 0;      // PID到位后连续稳定计数
static float g_move_position_tolerance = 0.0f; // 到位判定允许的误差：本次移动距离的一半
static uint8_t g_move_settling = 0;          // 已进入终点保持，禁止继续输出平移速度
 

static float g_move_speed_limit = DEFAULT_MOVE_SPEED_PULSES;
void position_set_move_speed(float speed) {g_move_speed_limit = speed;}
static float g_current_move_speed = 0.0f;   // 当前速度指令
static float g_active_position_kp = MOVE_POSITION_KP_NORTH_5_0; // 本段移动固定使用的位置P环增益
static float g_ramp_speed_limit = 0.0f;     // 缓加速产生的速度上限
static uint8_t g_ramp_active = 0;           // 缓加速进行中标志
#define RAMP_STEPS  30   // 从0缓加速到速度200的周期数（20*20ms=400ms）

// 车体系相对短距离运动：用于观察失败后的“当前视角后退/前进”。
// 普通 position_move(N/S/E/W) 按全局坐标目标停止；
// 这里按实际已经走过的距离停止，并且速度直接给车体系。
#define BODY_RELATIVE_NONE      (0)
#define BODY_RELATIVE_ACTIVE    (1)

static uint8_t g_body_relative_move = BODY_RELATIVE_NONE;
static float g_body_start_x = 0.0f;
static float g_body_start_y = 0.0f;
static float g_body_target_distance = 0.0f; // 车体系相对移动的目标格数
static float g_body_pulses_per_grid = PULSES_PER_GRID_NS; // 本段位置误差换算所用的每格脉冲数
static float g_body_cmd_vx = 0.0f;       // 车体系 X 速度方向，右为正
static float g_body_cmd_vy = 0.0f;       // 车体系 Y 速度方向，前为正



// 方向对应的全局速度向量
static const struct {
    float vx;   // X方向速度分量（归一化）
    float vy;   // Y方向速度分量（归一化）
} g_dir_vector[4] = {
    [NORTH] = {  0.0f,  1.0f },   // 向北：Y正方向
    [SOUTH] = {  0.0f, -1.0f },   // 向南：Y负方向
    [EAST]  = {  1.0f,  0.0f },   // 向东：X正方向
    [WEST]  = { -1.0f,  0.0f }    // 向西：X负方向
};

// ==================== 静态函数声明 ====================
static void update_motion_control(void);  // 定点移动控制更新
static void position_body_relative_move(float vx_body_unit, float vy_body_unit, float grids);
static float get_move_position_kp(uint8_t direction, float grids);
static float update_move_speed(float position_error);
static uint8_t update_move_settle(float position_error, float signed_speed);

void control_begin_translation(void);
void control_hold_translation(void);

static float get_move_position_kp(uint8_t direction, float grids)
{
    float kp_0_1;
    float kp_0_3;
    float kp_0_5;
    float kp_1_0;
    float kp_2_0;
    float kp_2_5;
    float kp_3_0;
    float kp_5_0;
    float kp_7_0;

    switch(direction)
    {
        case SOUTH:
            kp_0_1 = MOVE_POSITION_KP_SOUTH_0_1;
            kp_0_3 = MOVE_POSITION_KP_SOUTH_0_3;
            kp_0_5 = MOVE_POSITION_KP_SOUTH_0_5;
            kp_1_0 = MOVE_POSITION_KP_SOUTH_1_0;
            kp_2_0 = MOVE_POSITION_KP_SOUTH_2_0;
            kp_2_5 = MOVE_POSITION_KP_SOUTH_2_5;
            kp_3_0 = MOVE_POSITION_KP_SOUTH_3_0;
            kp_5_0 = MOVE_POSITION_KP_SOUTH_5_0;
            kp_7_0 = MOVE_POSITION_KP_SOUTH_7_0;
            break;

        case EAST:
            kp_0_1 = MOVE_POSITION_KP_EAST_0_1;
            kp_0_3 = MOVE_POSITION_KP_EAST_0_3;
            kp_0_5 = MOVE_POSITION_KP_EAST_0_5;
            kp_1_0 = MOVE_POSITION_KP_EAST_1_0;
            kp_2_0 = MOVE_POSITION_KP_EAST_2_0;
            kp_2_5 = MOVE_POSITION_KP_EAST_2_5;
            kp_3_0 = MOVE_POSITION_KP_EAST_3_0;
            kp_5_0 = MOVE_POSITION_KP_EAST_5_0;
            kp_7_0 = MOVE_POSITION_KP_EAST_7_0;
            break;

        case WEST:
            kp_0_1 = MOVE_POSITION_KP_WEST_0_1;
            kp_0_3 = MOVE_POSITION_KP_WEST_0_3;
            kp_0_5 = MOVE_POSITION_KP_WEST_0_5;
            kp_1_0 = MOVE_POSITION_KP_WEST_1_0;
            kp_2_0 = MOVE_POSITION_KP_WEST_2_0;
            kp_2_5 = MOVE_POSITION_KP_WEST_2_5;
            kp_3_0 = MOVE_POSITION_KP_WEST_3_0;
            kp_5_0 = MOVE_POSITION_KP_WEST_5_0;
            kp_7_0 = MOVE_POSITION_KP_WEST_7_0;
            break;

        case NORTH:
        default:
            kp_0_1 = MOVE_POSITION_KP_NORTH_0_1;
            kp_0_3 = MOVE_POSITION_KP_NORTH_0_3;
            kp_0_5 = MOVE_POSITION_KP_NORTH_0_5;
            kp_1_0 = MOVE_POSITION_KP_NORTH_1_0;
            kp_2_0 = MOVE_POSITION_KP_NORTH_2_0;
            kp_2_5 = MOVE_POSITION_KP_NORTH_2_5;
            kp_3_0 = MOVE_POSITION_KP_NORTH_3_0;
            kp_5_0 = MOVE_POSITION_KP_NORTH_5_0;
            kp_7_0 = MOVE_POSITION_KP_NORTH_7_0;
            break;
    }

    if(grids <= 0.1f)
    {
        return kp_0_1;
    }
    else if(grids < 0.3f)
    {
        return kp_0_1 +
               (grids - 0.1f) / (0.3f - 0.1f) *
               (kp_0_3 - kp_0_1);
    }
    else if(grids < 0.5f)
    {
        return kp_0_3 +
               (grids - 0.3f) / (0.5f - 0.3f) *
               (kp_0_5 - kp_0_3);
    }
    else if(grids < 1.0f)
    {
        return kp_0_5 +
               (grids - 0.5f) / (1.0f - 0.5f) *
               (kp_1_0 - kp_0_5);
    }
    else if(grids < 2.0f)
    {
        return kp_1_0 +
               (grids - 1.0f) / (2.0f - 1.0f) *
               (kp_2_0 - kp_1_0);
    }
    else if(grids < 2.5f)
    {
        return kp_2_0 +
               (grids - 2.0f) / (2.5f - 2.0f) *
               (kp_2_5 - kp_2_0);
    }
    else if(grids < 3.0f)
    {
        return kp_2_5 +
               (grids - 2.5f) / (3.0f - 2.5f) *
               (kp_3_0 - kp_2_5);
    }
    else if(grids < 5.0f)
    {
        return kp_3_0 +
               (grids - 3.0f) / (5.0f - 3.0f) *
               (kp_5_0 - kp_3_0);
    }
    else if(grids < 7.0f)
    {
        return kp_5_0 +
               (grids - 5.0f) / (7.0f - 5.0f) *
               (kp_7_0 - kp_5_0);
    }

    return kp_7_0;
}

static float update_move_speed(float position_error)
{
    float step;
    float position_speed_limit;

    if(g_ramp_active)
    {
        step = g_move_speed_limit / RAMP_STEPS;
        g_ramp_speed_limit += step;

        if(g_ramp_speed_limit >= g_move_speed_limit)
        {
            g_ramp_speed_limit = g_move_speed_limit;
            g_ramp_active = 0;
        }
    }
    else
    {
        g_ramp_speed_limit = g_move_speed_limit;
    }

    position_speed_limit = g_active_position_kp * position_error;
    
    
    // MOVE_SPEED_PULSES：最高速度上限
    // position_speed_limit：位置P环给出的速度上限
    // g_ramp_speed_limit：缓加速给出的速度上限
    g_current_move_speed = MIN(MIN(g_move_speed_limit,position_speed_limit),g_ramp_speed_limit);
    
    if(g_current_move_speed < 0.0f)
    {
        g_current_move_speed = 0.0f;
    }

    return g_current_move_speed;
}

static uint8_t update_move_settle(float position_error, float signed_speed)
{
    if(!g_move_settling)
    {
        /*
         * signed_speed沿本次指令方向为正。
         * 第一次进入允许停车区域且速度降到阈值时，立即撤销平移输出；
         * 即使采样时已经轻微过零为负，也会立刻阻止继续反推。
         */
        /* Once the target has been crossed, stop translation even if coast
         * carried the vehicle past the narrow approach window. Waiting for
         * fabs(error)<tolerance after overshoot leaves a zero-speed command
         * marked moving forever. The route pauses after the wheels settle
         * and leaves the measured endpoint unchanged. */
        if(position_error <= 0.0f ||
           (fabsf(position_error) < g_move_position_tolerance &&
            signed_speed <= MOVE_STOP_SPEED_THRESHOLD))
        {
            g_move_settling = 1;
            g_move_settle_count = 0;
            g_ramp_active = 0;
            g_ramp_speed_limit = 0.0f;
            g_current_move_speed = 0.0f;
            set_target_v(0.0f, 0.0f);
            control_hold_translation();
        }
    }

    if(g_move_settling)
    {
        set_target_v(0.0f, 0.0f);

        if(fabsf(signed_speed) <= MOVE_STOP_SPEED_THRESHOLD)
        {
            if(g_move_settle_count < MOVE_SETTLE_CYCLES)
            {
                g_move_settle_count++;
            }
        }
        else
        {
            g_move_settle_count = 0;
        }
    }

    return (g_move_settle_count >= MOVE_SETTLE_CYCLES);
}

void position_init(void)
{
    g_move_held=0;
    g_pos_x = 0.0f;
    g_pos_y = 0.0f;
    g_logic_x = 0.0f;
    g_logic_y = 0.0f;
    g_logic_target_x = 0.0f;
    g_logic_target_y = 0.0f;
    g_move_settle_count = 0;
    g_move_position_tolerance = 0.0f;
    g_move_settling = 0;
    g_current_move_speed = 0.0f;
    g_active_position_kp = MOVE_POSITION_KP_NORTH_3_0;
    g_ramp_speed_limit = 0.0f;
    g_ramp_active = 0;
    g_body_relative_move = BODY_RELATIVE_NONE;
    g_body_start_x = 0.0f;
    g_body_start_y = 0.0f;
    g_body_target_distance = 0.0f;
    g_body_pulses_per_grid = PULSES_PER_GRID_NS;
    g_body_cmd_vx = 0.0f;
    g_body_cmd_vy = 0.0f;
}

float position_get_x(void)
{
    return g_pos_x;
}

float position_get_y(void)
{
    return g_pos_y;
}

void position_reset(float x, float y)
{
    g_pos_x = x;
    g_pos_y = y;
    g_logic_x = x;
    g_logic_y = y;
    g_logic_target_x = x;
    g_logic_target_y = y;
    g_move_settle_count = 0;
    g_move_position_tolerance = 0.0f;
    g_move_settling = 0;
    g_current_move_speed = 0.0f;
    g_active_position_kp = MOVE_POSITION_KP_NORTH_3_0;
    g_ramp_speed_limit = 0.0f;
    g_ramp_active = 0;
    g_body_relative_move = BODY_RELATIVE_NONE;
    g_body_start_x = x;
    g_body_start_y = y;
    g_body_target_distance = 0.0f;
    g_body_pulses_per_grid = PULSES_PER_GRID_NS;
    g_body_cmd_vx = 0.0f;
    g_body_cmd_vy = 0.0f;
}

void position_reset_grid(float grid_x, float grid_y)
{
    position_reset(grid_x * PULSES_PER_GRID_EW,
                   grid_y * PULSES_PER_GRID_NS);
}

void position_get_grid(float *grid_x, float *grid_y)
{
    *grid_x = g_pos_x / PULSES_PER_GRID_EW;
    *grid_y = g_pos_y / PULSES_PER_GRID_NS;
}

static void position_body_relative_move(float vx_body_unit, float vy_body_unit, float grids)
{
    float yaw;
    float rad;
    float cos_yaw;
    float sin_yaw;
    float vx_global_unit;
    float vy_global_unit;
    uint8_t direction;

    if(grids <= 0.0f)
    {
        return;
    }

    g_body_relative_move = BODY_RELATIVE_ACTIVE;
    g_body_start_x = g_pos_x;
    g_body_start_y = g_pos_y;
    g_body_target_distance = grids;
    g_body_cmd_vx = vx_body_unit;
    g_body_cmd_vy = vy_body_unit;

    // 用启动瞬间航向估算逻辑目标，主要用于这段结束后 g_logic_x/y 连续。
    yaw = angle_get_current();
    rad = yaw * 3.14159f / 180.0f;
    cos_yaw = cosf(rad);
    sin_yaw = sinf(rad);

    vx_global_unit = -vy_body_unit * sin_yaw + vx_body_unit * cos_yaw;
    vy_global_unit =  vy_body_unit * cos_yaw + vx_body_unit * sin_yaw;

    if(fabsf(vx_global_unit) >= fabsf(vy_global_unit))
    {
        direction = (vx_global_unit >= 0.0f) ? EAST : WEST;
    }
    else
    {
        direction = (vy_global_unit >= 0.0f) ? NORTH : SOUTH;
    }

    g_body_pulses_per_grid =
        (direction == EAST || direction == WEST) ?
        PULSES_PER_GRID_EW : PULSES_PER_GRID_NS;
    g_move_position_tolerance =
        grids * g_body_pulses_per_grid * 0.5f;
    g_active_position_kp = get_move_position_kp(direction, grids);

    g_logic_target_x =
        g_logic_x + vx_global_unit * grids * PULSES_PER_GRID_EW;
    g_logic_target_y =
        g_logic_y + vy_global_unit * grids * PULSES_PER_GRID_NS;

    g_move_settle_count = 0;
    g_move_settling = 0;
    g_is_moving = 1;

    g_current_move_speed = 0.0f;
    g_ramp_speed_limit = 0.0f;
    g_ramp_active = 1;

    set_target_v(0.0f, 0.0f);
    control_begin_translation();
}

void position_move_body_backward(float grids)
{
    position_body_relative_move(0.0f, -1.0f, grids);
}

void position_move_body_forward(float grids)
{
    position_body_relative_move(0.0f, 1.0f, grids);
}

void position_move_body_left(float grids)
{
    position_body_relative_move(-1.0f, 0.0f, grids);
}

void position_move_body_right(float grids)
{
    position_body_relative_move(1.0f, 0.0f, grids);
}





//========定点移动控制函数==========

void position_move(uint8_t direction, float grids)
{
    float pulses_per_grid;
    float delta_x;
    float delta_y;

    if(grids <= 0) return;
    if(direction > WEST) return;

    pulses_per_grid =
        (direction == EAST || direction == WEST) ?
        PULSES_PER_GRID_EW : PULSES_PER_GRID_NS;

    // 依据本次命令的总路程选定位置KP，并在整段运动中保持不变。
    g_active_position_kp = get_move_position_kp(direction, grids);
    g_move_position_tolerance = MIN(grids * 0.5f, 0.10f) * pulses_per_grid;

    g_body_relative_move = BODY_RELATIVE_NONE;
    g_body_target_distance = 0.0f;
    g_body_pulses_per_grid = pulses_per_grid;
    g_body_cmd_vx = 0.0f;
    g_body_cmd_vy = 0.0f;
    
    // 记录移动方向
    g_current_direction = direction;
    
    // 计算目标坐标（脉冲数）
    delta_x = g_dir_vector[direction].vx * grids * PULSES_PER_GRID_EW;
    delta_y = g_dir_vector[direction].vy * grids * PULSES_PER_GRID_NS;
    
    g_logic_target_x = g_logic_x + delta_x;
    g_logic_target_y = g_logic_y + delta_y;

    // 目标坐标始终对齐到理论格点。
    // 例如上一段结束后实际比理论少走/多走，本段沿同一轴移动时会自动多走/少走补偿。
    // 注意：本段只修正当前移动方向所在的轴，不在垂直方向额外叠加速度。
    // 这里直接用 g_logic_target_x/y 作为本轴目标；实际位置 g_pos_x/y 与理论位置 g_logic_x/y 的差值会自然体现在本段实际移动距离中。
    if(direction == NORTH || direction == SOUTH)
    {
        g_target_x = g_pos_x;
        g_target_y = g_logic_target_y;
    }
    else
    {
        g_target_x = g_logic_target_x;
        g_target_y = g_pos_y;
    }

    g_move_settle_count = 0;
    g_move_settling = 0;
    g_is_moving = 1;
    
    // ===== 缓加速初始化 =====
    g_current_move_speed = 0.0f;           // 从0开始
    g_ramp_speed_limit = 0.0f;             // 缓加速上限从0开始
    g_ramp_active = 1;                     // 启动缓加速
    
    // 先设置速度为0，避免突然启动
    set_target_v(0.0f, 0.0f);
    control_begin_translation();
}

void position_get_target_grid(float *grid_x, float *grid_y)
{
    *grid_x=g_logic_target_x/PULSES_PER_GRID_EW;
    *grid_y=g_logic_target_y/PULSES_PER_GRID_NS;
}

/* Bypass targets are absolute; never restart the interrupted full distance. */
void position_move_to_grid(uint8_t direction, float grid_x, float grid_y)
{
    float remaining;
    if(direction>WEST) return;
    position_stop_move();
    g_logic_x=grid_x*PULSES_PER_GRID_EW;g_logic_y=grid_y*PULSES_PER_GRID_NS;
    g_logic_target_x=g_logic_x;g_logic_target_y=g_logic_y;
    if(direction==NORTH || direction==SOUTH) {
        remaining=(g_logic_y-g_pos_y)/PULSES_PER_GRID_NS;
        if(direction==SOUTH) remaining=-remaining;
        g_logic_y=g_pos_y;
    } else {
        remaining=(g_logic_x-g_pos_x)/PULSES_PER_GRID_EW;
        if(direction==WEST) remaining=-remaining;
        g_logic_x=g_pos_x;
    }
    if(remaining>0.0f) position_move(direction,remaining);
    else {g_logic_x=g_logic_target_x;g_logic_y=g_logic_target_y;}
}

/* Preserve the original absolute target and logical route origin. */
void position_hold_move(uint8_t hold)
{
    if(hold) {
        g_move_held=1;set_target_v(0.0f,0.0f);
    } else if(g_move_held) {
        g_move_held=0;
        if(g_is_moving) {
            g_current_move_speed=0.0f;g_ramp_speed_limit=0.0f;g_ramp_active=1;
            control_begin_translation();
        }
    }
}

void position_stop_move(void)
{
    g_move_held=0;
    if(g_is_moving) {
        g_is_moving = 0;
        g_ramp_active = 0;           // 重置缓加速标志
        g_ramp_speed_limit = 0.0f;
        g_current_move_speed = 0.0f;
        g_move_settle_count = 0;
        g_move_position_tolerance = 0.0f;
        g_move_settling = 0;
        g_body_relative_move = BODY_RELATIVE_NONE;
        g_body_target_distance = 0.0f;
        g_body_pulses_per_grid = PULSES_PER_GRID_NS;
        g_body_cmd_vx = 0.0f;
        g_body_cmd_vy = 0.0f;
        set_target_v(0.0f, 0.0f);
        control_hold_translation();
    }
}

uint8_t position_is_moving(void)
{
    return g_is_moving;
}


void position_update(void)
{
    // 1. 获取本体坐标系下的速度（从control.c获取）
    float vx_body = get_body_vx();  // 右+ 单位：脉冲数/20ms
    float vy_body = get_body_vy();  // 前+ 单位：脉冲数/20ms
    
    static float vx_total=0.0f;
    static float vy_total=0.0f;
    
    vx_total+=vx_body;
    vy_total+=vy_body;
    
    // 2. 获取当前航向角（从angle.c获取）
    float yaw = angle_get_current();  // 单位：度，0为正前方，逆时针增加
    
    // 3. 坐标系变换：本体速度 → 全局速度
    //    注意：C语言的sin/cos接受弧度，需要转换
    float rad = yaw * 3.14159f / 180.0f;
    float cos_yaw = cosf(rad);
    float sin_yaw = sinf(rad);
    
    float vx_body_grid = vx_body / PULSES_PER_GRID_EW;
    float vy_body_grid = vy_body / PULSES_PER_GRID_NS;

    float vy_global_grid =
        vy_body_grid * cos_yaw + vx_body_grid * sin_yaw;
    float vx_global_grid =
        -vy_body_grid * sin_yaw + vx_body_grid * cos_yaw;

    float vx_global = vx_global_grid * PULSES_PER_GRID_EW;
    float vy_global = vy_global_grid * PULSES_PER_GRID_NS;
    

    
    // 4. 积分累加：速度的定义是20ms内的脉冲数，直接累加速度就是位移（不需要乘以时间！！！！）
    g_pos_x += vx_global;
    g_pos_y += vy_global;
    pynq_telemetry_sample(vx_body_grid, vy_body_grid,
                          g_pos_x / PULSES_PER_GRID_EW,
                          g_pos_y / PULSES_PER_GRID_NS, yaw);
    
    //printf("g_pos_x = %f \r\n", g_pos_x);
    //printf("g_pos_y = %f \r\n\r\n", g_pos_y);
          /*
          seekfree_assistant_oscilloscope_data.data[0] = g_pos_x;
          seekfree_assistant_oscilloscope_data.data[1] = g_pos_y;
          seekfree_assistant_oscilloscope_data.channel_num = 2;
          seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
          seekfree_assistant_data_analysis();
          */
          
    //5. 定点移动控制更新
    update_motion_control();
}


/**
 * @brief 定点移动控制更新（在20ms中断中调用，由 position_update 调用）
 * 
 * 功能：
 * 1. 如果不在移动状态，直接返回
 * 2. 计算当前位置到目标点的距离
 * 3. 如果距离小于阈值，停止移动
 * 4. 否则，计算指向目标点的方向角，设置 vx, vy 使车向目标点移动
 *    同时让角度闭环保持车头朝北（目标角度0度）
 */
static void update_motion_control(void)
{
    float error = 0.0f;
    float signed_speed = 0.0f;
    float vx_cmd = 0.0f;
    float vy_cmd = 0.0f;
    float rad = angle_get_current() * 0.017453292519943295f;
    float c = cosf(rad), s = sinf(rad);
    float bx = get_body_vx() / PULSES_PER_GRID_EW;
    float by = get_body_vy() / PULSES_PER_GRID_NS;
    float world_vx = (bx*c - by*s) * PULSES_PER_GRID_EW;
    float world_vy = (by*c + bx*s) * PULSES_PER_GRID_NS;

    if(!g_is_moving || g_move_held) return;

    // 车体系短距离运动：不要再走下面的全局 N/S/E/W 停止判断。
    // 只看从启动点到当前位置的实际位移长度，速度始终由正常位置P环控制。
    if(g_body_relative_move == BODY_RELATIVE_ACTIVE)
    {
        float dx = g_pos_x - g_body_start_x;
        float dy = g_pos_y - g_body_start_y;
        float dx_grid = dx / PULSES_PER_GRID_EW;
        float dy_grid = dy / PULSES_PER_GRID_NS;
        float travelled =
            sqrtf(dx_grid * dx_grid + dy_grid * dy_grid);

        error =
            (g_body_target_distance - travelled) *
            g_body_pulses_per_grid;

        signed_speed =
            g_body_cmd_vx * get_body_vx() +
            g_body_cmd_vy * get_body_vy();

        if(update_move_settle(error, signed_speed))
        {
            g_logic_x = g_logic_target_x;
            g_logic_y = g_logic_target_y;
            position_stop_move();
            return;
        }

        if(g_move_settling)
        {
            return;
        }

        g_current_move_speed = update_move_speed(error);

        vx_cmd = g_body_cmd_vx * g_current_move_speed;
        vy_cmd = g_body_cmd_vy * g_current_move_speed;
        set_target_v(vx_cmd, vy_cmd);
        return;
    }

    // ===== 普通全局定点移动：停止判断只判断前进方向 =====
    switch(g_current_direction)
    {
        case NORTH:
            error = g_target_y - g_pos_y;
            signed_speed = world_vy;
            break;
        case SOUTH:
            error = g_pos_y - g_target_y;
            signed_speed = -world_vy;
            break;
        case EAST:
            error = g_target_x - g_pos_x;
            signed_speed = world_vx;
            break;
        case WEST:
            error = g_pos_x - g_target_x;
            signed_speed = -world_vx;
            break;
        default: return;
    }

    if(update_move_settle(error, signed_speed))
    {
        g_logic_x = g_logic_target_x;
        g_logic_y = g_logic_target_y;
        position_stop_move();
        return;
    }

    if(g_move_settling)
    {
        return;
    }

    g_current_move_speed = update_move_speed(error);

    switch(g_current_direction)
    {
        case NORTH: vy_cmd = g_current_move_speed; break;
        case SOUTH: vy_cmd = -g_current_move_speed; break;
        case EAST:  vx_cmd = g_current_move_speed; break;
        case WEST:  vx_cmd = -g_current_move_speed; break;
        default: return;
    }
    {
        float gx = vx_cmd / PULSES_PER_GRID_EW;
        float gy = vy_cmd / PULSES_PER_GRID_NS;
        set_target_v((gx*c + gy*s) * PULSES_PER_GRID_EW,
                     (gy*c - gx*s) * PULSES_PER_GRID_NS);
    }
}
