/* Offline V7 two-lap route. The national planner is not called in this build. */
#include "zf_common_headfile.h"
#include "main.h"
#include "angle.h"
#include "control.h"
#include "position.h"
#include "pynq_telemetry.h"
#include "route_board.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    interrupt_global_disable();
    gpio_init(LED1,GPO,GPIO_HIGH,GPO_PUSH_PULL);
    /* Establish zero PWM before any IMU wait/calibration. */
    pwm_init(MOTOR1_PWM,17000,0);gpio_init(MOTOR1_DIR,GPO,MOTOR1_FORWARD_DIR_LEVEL,GPO_PUSH_PULL);
    pwm_init(MOTOR2_PWM,17000,0);gpio_init(MOTOR2_DIR,GPO,MOTOR2_FORWARD_DIR_LEVEL,GPO_PUSH_PULL);
    pwm_init(MOTOR3_PWM,17000,0);gpio_init(MOTOR3_DIR,GPO,MOTOR3_FORWARD_DIR_LEVEL,GPO_PUSH_PULL);
    pwm_init(MOTOR4_PWM,17000,0);gpio_init(MOTOR4_DIR,GPO,MOTOR4_FORWARD_DIR_LEVEL,GPO_PUSH_PULL);
    control_init();control_stop_all();position_init();route_board_init();
    uart_init(UART_1,115200,UART1_TX_B12,UART1_RX_B13);
    while(imu660rb_init()) {gpio_toggle_level(LED1);system_delay_ms(100);}
    angle_init();
    encoder_quad_init(ENCODER_1,ENCODER_1_A,ENCODER_1_B);
    encoder_quad_init(ENCODER_2,ENCODER_2_A,ENCODER_2_B);
    encoder_quad_init(ENCODER_3,ENCODER_3_A,ENCODER_3_B);
    encoder_quad_init(ENCODER_4,ENCODER_4_A,ENCODER_4_B);
    encoder_clear_count(ENCODER_1);encoder_clear_count(ENCODER_2);
    encoder_clear_count(ENCODER_3);encoder_clear_count(ENCODER_4);
    uart_rx_interrupt(UART_1,ZF_ENABLE);
    interrupt_set_priority(LPUART1_IRQn,1);
    pit_ms_init(PIT_CH0,5);pit_ms_init(PIT_CH1,20);
    interrupt_set_priority(PIT_IRQn,0);
    interrupt_global_enable(0);
    while(1) {pynq_telemetry_poll();route_board_poll();}
}
