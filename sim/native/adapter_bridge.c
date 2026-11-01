/* Python UART peer linked to actual position/PID; hardware is stubbed. */
#define main legacy_adapter_test
#include "adapter_host_test.c"
#undef main
void bridge_init(void) {
    unsigned i;control_init();position_init();route_board_init();yaw=target=0;seq=0;ticks=0;
    memset(duty,0,sizeof(duty));memset(status,0,sizeof(status));
    for(i=0;i<6;i++)tick(0);
}
void bridge_feed(const uint8_t *packet,unsigned length) {unsigned i;for(i=0;i<length;i++)route_board_rx(packet[i]);}
void bridge_tick(void) {tick(0);}
void bridge_status(uint8_t *packet) {memcpy(packet,status,18);}
float bridge_x(void) {return position_get_x()/PULSES_PER_GRID_EW;}
float bridge_y(void) {return position_get_y()/PULSES_PER_GRID_NS;}
float bridge_goal_x(void) {float x,y;position_get_target_grid(&x,&y);return x;}
float bridge_goal_y(void) {float x,y;position_get_target_grid(&x,&y);return y;}
float bridge_motor_peak(void) {
    float peak=0;unsigned i;for(i=0;i<4;i++)if(fabsf(g_motors[i].target_speed)>peak)peak=fabsf(g_motors[i].target_speed);return peak;
}
