/* Actual controller/adapter/position/PID, ideal encoder response only. */
#define main legacy_adapter_test
#include "adapter_host_test.c"
#undef main

static void configured8(uint8_t op,uint16_t value) {
    uint8_t p[12]={0xc7,0x7c,8,0,123,0,0,0,0,0,0,0};unsigned i;uint16_t crc;
    p[3]=op;p[8]=(uint8_t)value;p[9]=(uint8_t)(value>>8);crc=rv_crc16(p,10);
    p[10]=(uint8_t)crc;p[11]=(uint8_t)(crc>>8);
    for(i=0;i<12;i++)route_board_rx(p[i]);
    tick(0);
}
static void until_state(uint8_t state) {
    unsigned i;
    for(i=0;i<20000u && status[3]!=state;i++)tick(1);
    assert(status[3]==state);
}
static void start8(uint8_t direction,uint16_t distance) {
    unsigned i;
    control_init();position_init();route_board_init();yaw=target=0;seq=0;ticks=0;
    memset(duty,0,sizeof(duty));memset(status,0,sizeof(status));
    for(i=0;i<6;i++)tick(0);
    assert(status[2]==8);
    configured8(RV_ARM,1);configured8(RV_CFG_BEGIN,1);configured8(RV_CFG_SPEED,80);
    configured8(RV_CFG_MOVE,(uint16_t)((direction<<14)|distance));configured8(RV_CFG_HEADING,0);
    configured8(RV_CFG_COMMIT,0x1234);seq=0x1234;send(RV_START);until_state(RV_MOVE);
    for(i=0;i<10;i++)tick(1);
}
static void hold(void) {send(RV_HOLD);until_state(RV_HELD);zero();}
int main(void) {
    unsigned d,i,j;float original_x,original_y,x,y,side;
    for(d=0;d<4;d++) {
        unsigned phases[3]={0,0,0};
        start8((uint8_t)d,1000);position_get_target_grid(&original_x,&original_y);hold();
        send(RV_BYPASS);until_state(RV_BYPASS_RIGHT);side=0;
        for(i=0;i<20000u && status[3]!=RV_DONE;i++) {
            tick(1);position_get_grid(&x,&y);
            if(d<2) {if(fabsf(x)>side)side=fabsf(x);}else if(fabsf(y)>side)side=fabsf(y);
            if(status[3]>=RV_BYPASS_RIGHT && status[3]<=RV_BYPASS_LEFT) {
                unsigned phase=status[3]-RV_BYPASS_RIGHT;
                assert(status[10]==0);
                if(!phases[phase]) {
                    phases[phase]=1;hold();assert(status[11]==0x10u+phase);
                    for(j=0;j<10;j++){tick(0);zero();}
                    send(RV_RELEASE);until_state((uint8_t)(RV_BYPASS_RIGHT+phase));
                }
            }
        }
        assert(status[3]==RV_DONE && phases[0] && phases[1] && phases[2]);zero();
        position_get_grid(&x,&y);assert(fabsf(x-original_x)<.2f && fabsf(y-original_y)<.2f);
        position_get_target_grid(&x,&y);assert(fabsf(x-original_x)<.001f && fabsf(y-original_y)<.001f);
        assert(side>2.3f && side<2.7f);
        printf("BYPASS_DIRECTION_%u_OK goal=(%.3f,%.3f) side=%.3f grid\n",d,original_x,original_y,side);
    }
    for(d=RV_BYPASS_RIGHT;d<=RV_BYPASS_LEFT;d++) {
        start8(NORTH,1000);hold();send(RV_BYPASS);until_state((uint8_t)d);
        send(RV_STOP);tick(0);zero();until_state(RV_STOPPED);
        send(RV_RELEASE);send(RV_BYPASS);
        for(i=0;i<100;i++){tick(1);zero();assert(status[3]==RV_STOPPED);}
    }
    start8(NORTH,400);hold();send(RV_BYPASS);
    for(i=0;i<20;i++){tick(1);zero();}
    assert(status[3]==RV_HELD && status[11]==RV_BYPASS_SHORT);
    send(RV_RELEASE);until_state(RV_DONE);assert(status[11]==RV_OK);zero();
    start8(NORTH,1000);hold();send(RV_BYPASS);send(RV_HOLD);
    for(i=0;i<20;i++){tick(1);zero();assert(status[3]==RV_HELD);}
    send(RV_BYPASS);until_state(RV_BYPASS_RIGHT);send(RV_STOP);tick(0);zero();
    puts("BYPASS_ADAPTER_OK: four axes, preserved target, all holds, all STOPs, short-step fallback, same-burst HOLD priority");
    return 0;
}
