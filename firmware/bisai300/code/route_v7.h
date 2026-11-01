#ifndef ROUTE_V7_H
#define ROUTE_V7_H
#include <stdint.h>

enum { RV_WAIT, RV_ARMED, RV_COUNTDOWN, RV_MOVE, RV_GAP, RV_TURN,
       RV_DONE, RV_STOPPED, RV_FAULT, RV_PAUSED, RV_HOLDING, RV_HELD,
       RV_BYPASS_RIGHT, RV_BYPASS_FORWARD, RV_BYPASS_LEFT };
enum { RV_STOP=0, RV_ARM=1, RV_START=2, RV_HEARTBEAT=3, RV_CONTINUE=4,
       RV_HOLD=5, RV_RELEASE=6, RV_BYPASS=7, RV_CONTINUE_SPEED=0x60 };
/* V3 config packets keep the 12-byte wire format. Sequence carries the
 * parameter value; session and CRC still bind every configuration packet. */
enum { RV_CFG_BEGIN=0x10, RV_CFG_SPEED=0x11, RV_CFG_COMMIT=0x12, RV_CFG_TURN_SPEED=0x13,
       RV_CFG_MOVE=0x20, RV_CFG_HEADING=0x30, RV_CFG_TURN=0x40, RV_CFG_SPIN=0x50 };
enum { RV_OK, RV_LINK_LOST, RV_TIMEOUT, RV_STALL, RV_POSE_ERROR,
       RV_HEADING_ERROR, RV_NOT_STILL, RV_BAD_SENSOR, RV_BYPASS_SHORT, RV_BYPASS_NOT_MOVE };
enum { RV_NO_ACTION, RV_MOVE_ACTION, RV_TURN_ACTION, RV_TARGET_ACTION };
typedef struct {
    uint8_t kind, direction, lap, leg;
    float heading, x, y, grids, turn_degrees;
} rv_step_t;
typedef struct {
    float x, y, yaw, wheel_peak;
    uint8_t moving;
    float goal_x, goal_y;
} rv_input_t;
typedef struct {
    uint8_t state, fault, step, action, reset_origin, resume_requested;
    uint16_t sequence, stable_ticks;
    uint32_t session, ticks, stage_at;
    uint8_t rx[12], rx_length;
    uint8_t protocol, count, speed, config_ack, config_ready, speed_set;
    uint16_t configured_actions, configured_headings;
    rv_step_t plan[12];
    float turn_previous_yaw, turn_progress, target_heading;
    uint8_t held_state, move_speed, next_speed, turn_speed;
    uint16_t held_stable_ticks;
    uint32_t held_at;
    uint8_t bypass_requested, bypass_active, command_direction, saved_move_speed;
    float bypass_x, bypass_y, bypass_goal_x, bypass_goal_y, command_x, command_y;
} rv_controller_t;

extern const rv_step_t rv_plan[12];
uint16_t rv_crc16(const uint8_t *data, unsigned length);
void rv_init(rv_controller_t *r);
void rv_command(rv_controller_t *r, uint8_t op, uint32_t session, uint16_t seq);
void rv_feed(rv_controller_t *r, uint8_t byte);
void rv_tick(rv_controller_t *r, const rv_input_t *in);
uint8_t rv_motor_enabled(const rv_controller_t *r);
void rv_status(const rv_controller_t *r, uint8_t packet[18]);
#endif
