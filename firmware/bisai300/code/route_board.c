#include "route_board.h"
#include "route_v7.h"
#include "zf_common_headfile.h"
#include "position.h"
#include "angle.h"
#include "control.h"
#include <math.h>

static rv_controller_t route;
static volatile uint8_t ring[128], head, tail;
static uint32_t last_status;
static float turn_command;
void route_board_init(void) {rv_init(&route);head=tail=0;last_status=0;turn_command=0.0f;}
void route_board_rx(unsigned char b) {
    uint8_t next=(uint8_t)((head+1u)&127u);
    if(next==tail) return;
    ring[head]=b;head=next;
}
void route_board_20ms(void) {
    rv_input_t in;
    unsigned i,n=0;
    float vz,vx,vy,turn_limit;
    while(tail!=head && n++<64u) {
        uint8_t b=ring[tail];tail=(uint8_t)((tail+1u)&127u);rv_feed(&route,b);
    }
    in.x=position_get_x()/PULSES_PER_GRID_EW;
    in.y=position_get_y()/PULSES_PER_GRID_NS;
    position_get_target_grid(&in.goal_x,&in.goal_y);
    in.yaw=angle_get_current();in.moving=position_is_moving();in.wheel_peak=0.0f;
    for(i=0;i<4;i++) {
        float v=fabsf(control_get_current_speed((uint8_t)i));
        if(v>in.wheel_peak) in.wheel_peak=v;
    }
    rv_tick(&route,&in);
    if(route.reset_origin) {position_init();angle_reset_current(0.0f);}
    if(route.action) {
        angle_set_target(route.plan[route.step].heading);
        position_set_move_speed((float)route.move_speed);
        if(route.action==RV_MOVE_ACTION) position_move(route.plan[route.step].direction,route.plan[route.step].grids);
        else if(route.action==RV_TARGET_ACTION) position_move_to_grid(route.command_direction,route.command_x,route.command_y);
        else {position_stop_move();set_target_v(0.0f,0.0f);control_hold_translation();}
    }
    if(route.state==RV_HOLDING || route.state==RV_HELD) {
        turn_command=0.0f;position_hold_move(1);set_target_v(0.0f,0.0f);control_stop_all();return;
    }
    position_hold_move(0);
    if(!rv_motor_enabled(&route)) {
        turn_command=0.0f;position_stop_move();set_target_v(0.0f,0.0f);control_stop_all();return;
    }
    if(route.state==RV_TURN) angle_set_target(route.target_heading);
    vz=angle_calc_vz();
    /* Configured angular limit only affects pure rotation, not move heading hold. */
    turn_limit=route.state==RV_TURN?(float)route.turn_speed:40.0f;
    if(vz>turn_limit) vz=turn_limit;
    if(vz< -turn_limit) vz= -turn_limit;
    if(route.state==RV_TURN) {
        /* Ramp acceleration only, 8 pulses/20ms per control tick. Deceleration,
         * HOLD and STOP remain immediate; leave the heading PID unchanged. */
        if(vz*turn_command<0.0f) turn_command=0.0f;
        turn_limit=fabsf(turn_command)+8.0f;
        if(vz>turn_limit) vz=turn_limit;
        if(vz< -turn_limit) vz= -turn_limit;
        turn_command=vz;
    } else turn_command=0.0f;
    vx=get_target_vx();vy=get_target_vy();
    control_set_all_target_speed(vy+vx+vz,vy-vx-vz,vy-vx+vz,vy+vx-vz);
    control_run_speed_pid();
}
void route_board_poll(void) {
    uint8_t packet[18];
    uint32_t saved=__get_PRIMASK();
    __disable_irq();
    if((uint32_t)(route.ticks-last_status)<5u) {__set_PRIMASK(saved);return;}
    last_status=route.ticks;rv_status(&route,packet);__set_PRIMASK(saved);
    uart_write_buffer(UART_1,packet,sizeof(packet));
}
