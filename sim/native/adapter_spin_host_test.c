#define main legacy_adapter_test
#include "adapter_host_test.c"
#undef main
static void configured(uint8_t op,uint16_t value) {
    uint8_t p[12]={0xc7,0x7c,4,0,123,0,0,0,0,0,0,0};unsigned k;uint16_t crc;
    p[3]=op;p[8]=(uint8_t)value;p[9]=(uint8_t)(value>>8);crc=rv_crc16(p,10);
    p[10]=(uint8_t)crc;p[11]=(uint8_t)(crc>>8);for(k=0;k<12;k++)route_board_rx(p[k]);tick(0);
}
int main(void) {
    unsigned i;float previous=0,total=0,difference;
    control_init();position_init();route_board_init();yaw=target=0;seq=0;ticks=0;
    memset(duty,0,sizeof(duty));memset(status,0,sizeof(status));
    configured(RV_ARM,1);configured(RV_CFG_BEGIN,1);configured(RV_CFG_SPEED,80);
    configured(RV_CFG_SPIN,7200);configured(RV_CFG_COMMIT,0x1234);seq=0x1234;send(RV_START);
    for(i=0;i<20000 && status[3]!=RV_DONE;i++) {
        tick(1);difference=yaw-previous;while(difference>180)difference-=360;while(difference<=-180)difference+=360;
        total-=difference;previous=yaw;
    }
    assert(status[3]==RV_DONE);assert(total>719&&total<721);zero();
    printf("ADAPTER_FULL_TWO_TURNS_OK: measured ideal-model clockwise rotation %.2f degrees\n",total);
    return 0;
}
