#define main legacy_adapter_test
#include "adapter_host_test.c"
#undef main
static void configured(uint8_t op,uint16_t value) {
    uint8_t p[12]={0xc7,0x7c,3,0,123,0,0,0,0,0,0,0};unsigned k;uint16_t crc;
    p[3]=op;p[8]=(uint8_t)value;p[9]=(uint8_t)(value>>8);crc=rv_crc16(p,10);
    p[10]=(uint8_t)crc;p[11]=(uint8_t)(crc>>8);for(k=0;k<12;k++)route_board_rx(p[k]);tick(0);
}
int main(void) {
    unsigned i,j;control_init();position_init();route_board_init();yaw=target=0;seq=0;ticks=0;
    memset(duty,0,sizeof(duty));memset(status,0,sizeof(status));
    configured(RV_ARM,1);configured(RV_CFG_BEGIN,3);configured(RV_CFG_SPEED,40);
    configured(RV_CFG_MOVE,200);configured(RV_CFG_HEADING,0);
    configured(RV_CFG_TURN+1,(uint16_t)(int16_t)-4500);
    configured(RV_CFG_MOVE+2,(uint16_t)((2u<<14)|100u));configured(RV_CFG_HEADING+2,(uint16_t)(int16_t)-4500);
    configured(RV_CFG_COMMIT,0x1234);seq=0x1234;send(RV_START);
    for(i=0;i<20000 && status[3]!=RV_DONE;i++) {
        tick(1);
        for(j=0;j<4;j++)assert(fabsf(g_motors[j].target_speed)<=81);
        if(status[3]==RV_PAUSED){send(RV_CONTINUE);for(j=0;j<6;j++)tick(1);}
    }
    assert(status[3]==RV_DONE&&status[10]==2);zero();
    puts("ADAPTER_CONFIGURED_RECIPE_OK: actual position/control code, 2-grid move, -45deg turn, 1-grid lateral move, reduced speed");
    return 0;
}
