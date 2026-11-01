#ifndef MAIN_H
#define MAIN_H

#include "zf_common_headfile.h"

#define PIT_CH                         (PIT_CH0 )                                 // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用

// 编码器引脚信息
// 请确保两轮编码器前进时回传值都为正数，若回传为负请isr.c中encoder_get_count前修改正负号
#define ENCODER_1                   (QTIMER1_ENCODER1)  // 注意拼写
#define ENCODER_1_A                 (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_B                 (QTIMER1_ENCODER1_CH2_C1)

#define ENCODER_2                   (QTIMER1_ENCODER2)
#define ENCODER_2_A                 (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_B                 (QTIMER1_ENCODER2_CH2_C24)

#define ENCODER_3                   (QTIMER2_ENCODER1)
#define ENCODER_3_A                 (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_3_B                 (QTIMER2_ENCODER1_CH2_C4)

#define ENCODER_4                   (QTIMER2_ENCODER2)
#define ENCODER_4_A                 (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_4_B                 (QTIMER2_ENCODER2_CH2_C25)

// 电机参数设置
// !!! 请确保dir和pwm脚不要弄反了，否则电机会疯转 !!!   

// 电机1：左前轮（定义摄像头的朝向是前方）
#define MOTOR1_DIR               (C9 )
#define MOTOR1_PWM               (PWM2_MODULE1_CHA_C8)
// 电机前进需要的DIR脚的电平为低电平，下同
#define MOTOR1_FORWARD_DIR_LEVEL (GPIO_LOW)

// 电机2：右前轮
#define MOTOR2_DIR               (C7 )
#define MOTOR2_PWM               (PWM2_MODULE0_CHA_C6)
#define MOTOR2_FORWARD_DIR_LEVEL (GPIO_LOW)

// 电机3：左后轮
#define MOTOR3_DIR               (D2 )
#define MOTOR3_PWM               (PWM2_MODULE3_CHB_D3)
#define MOTOR3_FORWARD_DIR_LEVEL (GPIO_LOW)

// 电机4：右后轮
#define MOTOR4_DIR               (C10 )
#define MOTOR4_PWM               (PWM2_MODULE2_CHB_C11)
#define MOTOR4_FORWARD_DIR_LEVEL (GPIO_LOW)


// 添加LED
#define LED1                     (B9)


// UART 引脚（与学长一致）
#define UART1_TX_PIN    UART1_TX_B12
#define UART1_RX_PIN    UART1_RX_B13

#endif
