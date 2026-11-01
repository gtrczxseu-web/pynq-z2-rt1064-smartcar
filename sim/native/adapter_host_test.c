/* Ideal-wheel host model, NOT a real floor/motor test. Links the actual
 * national PID/control/position and V7 route adapter, stubbing only hardware
 * and the IMU angular response. */
#include "route_board.h"
#include "route_v7.h"
#include "control.h"
#include "position.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static float yaw,target;
static int duty[4];
static uint8_t status[18];
static uint16_t seq;
static unsigned ticks,seen[12];
uint32_t __get_PRIMASK(void) {return 0;}
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t v) {(void)v;}
void gpio_set_level(int p,int l) {(void)p;(void)l;}
void pwm_set_duty(int p,int d) {assert(p>=0&&p<4);duty[p]=d;}
void uart_write_buffer(int u,const uint8_t *p,uint32_t n) {(void)u;assert(n==18);memcpy(status,p,n);}
float angle_get_current(void) {return yaw;}
void angle_set_target(float a) {target=a;}
void angle_reset_current(float a) {yaw=a;}
float angle_calc_vz(void) {
    float e=yaw-target;
    while(e>180)e-=360;
    while(e<=-180)e+=360;
    if(fabsf(e)<.2f)return 0;
    return e*2.5f;
}
void pynq_telemetry_sample(float a,float b,float c,float d,float e) {(void)a;(void)b;(void)c;(void)d;(void)e;}
static void send(uint8_t op) {
    uint8_t p[12]={0xc7,0x7c,2,0,123,0,0,0,0,0,0,0};unsigned i;uint16_t crc;
    ++seq;p[3]=op;p[8]=(uint8_t)seq;p[9]=(uint8_t)(seq>>8);crc=rv_crc16(p,10);
    p[10]=(uint8_t)crc;p[11]=(uint8_t)(crc>>8);
    for(i=0;i<12;i++)route_board_rx(p[i]);
}
static void tick(int heartbeat) {
    int16_t enc[4];unsigned i;float turn;
    if(heartbeat && ticks%5u==0u)send(RV_HEARTBEAT);
    turn=(g_motors[0].target_speed-g_motors[1].target_speed+
          g_motors[2].target_speed-g_motors[3].target_speed)/4;
    yaw-=turn*.026f;while(yaw>180)yaw-=360;while(yaw<=-180)yaw+=360;
    for(i=0;i<4;i++)enc[i]=(int16_t)g_motors[i].target_speed;
    control_update_speed(enc);position_update();route_board_20ms();route_board_poll();ticks++;
    if(status[3]==RV_MOVE||status[3]==RV_TURN)seen[status[10]]=1;
}
static void zero(void) {unsigned i;for(i=0;i<4;i++){assert(duty[i]==0);assert(g_motors[i].target_speed==0);}}
static void setup(void) {
    unsigned i;control_init();position_init();route_board_init();yaw=target=10;seq=0;ticks=0;
    memset(duty,0,sizeof(duty));memset(status,0,sizeof(status));memset(seen,0,sizeof(seen));
    for(i=0;i<31;i++){tick(0);zero();}
    send(RV_ARM);tick(0);send(RV_START);tick(0);
}
int main(void) {
    unsigned i,j;float x,y;
    setup();
    for(i=0;i<10000 && status[3]!=RV_DONE && status[3]!=RV_FAULT;i++) {
        tick(1);
        if(status[3]==RV_PAUSED) {
            uint8_t completed=status[10];
            for(j=0;j<60;j++){tick(0);zero();assert(status[3]==RV_PAUSED && status[10]==completed);}
            send(RV_CONTINUE);
            for(j=0;j<6 && status[3]==RV_PAUSED;j++)tick(1);
            assert(status[3]==RV_MOVE || status[3]==RV_TURN);
        }
    }
    assert(status[3]==RV_DONE);
    for(i=0;i<12;i++)assert(seen[i]);
    position_get_grid(&x,&y);assert(fabsf(x)<.25f&&fabsf(y)<.25f&&fabsf(yaw)<1);
    yaw+=10;for(i=0;i<100;i++){tick(1);zero();} /* no post-completion heading rotation */
    printf("ADAPTER_TWO_LAP_OK: ideal model ends x=%.4f y=%.4f grid, all 12 actions observed\n",x,y);
    setup();for(i=0;i<200;i++)tick(1);send(RV_STOP);tick(0);zero();
    for(i=0;i<100;i++){tick(1);zero();}
    setup();for(i=0;i<200;i++)tick(1);for(i=0;i<45;i++)tick(0);assert(status[3]==RV_MOVE);
    setup();for(i=0;i<200;i++)tick(1);for(i=0;i<200;i++)route_board_rx(0x55);tick(0);assert(status[3]==RV_MOVE);
    send(RV_STOP);for(i=0;i<2;i++)tick(0);zero();
    puts("ADAPTER_BASIC_MOTION_OK: manual STOP and finish are OFF; lease loss/RX overflow do not abort");
    return 0;
}
