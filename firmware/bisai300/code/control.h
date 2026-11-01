#ifndef CONTROL_H
#define CONTROL_H

#include "zf_common_headfile.h"
#include "pid.h"

// ==================== 控制参数 ====================
#define CONTROL_PERIOD_MS           (20)        // 控制周期 20ms
#define CONTROL_PERIOD_S            (0.02f)     // 控制周期 20ms = 0.02s

// ==================== 数据结构 ====================

/**
 * @brief 电机控制数据结构体
 * 
 * 速度单位说明：
 * - target_speed: 目标速度，单位是"脉冲数/10ms"
 * - current_speed: 当前实际速度，单位是"脉冲数/10ms"
 * 
 * 例如：target_speed = 150 表示希望在每个10ms周期内编码器产生150个脉冲
 * 这个值直接与编码器读数比较，没有转换误差
 * 
 * 控制周期：10ms
 * PWM输出范围：-50.0% ~ +50.0%
 */
typedef struct {
    // 输入
    float target_speed;          // 目标速度 (脉冲数/10ms)
    
    // 输出
    float current_speed;         // 当前实际速度 (脉冲数/10ms)
    float pwm_duty;              // 输出PWM占空比 (-50.0 ~ 50.0)
    
    // 原始数据
    int16_t encoder_pulse;        // 本次周期采集的脉冲数
    int32_t encoder_total;        // 总脉冲数（可选，用于位置计算）
    
    // PID控制器
    pid_param_t pid;              // 速度PID参数
} motor_control_t;

// ==================== 全局变量声明 ====================
extern motor_control_t g_motors[4];

/* Public stop/hold interfaces used by the V7 route adapter. */
void control_begin_translation(void);
void control_hold_translation(void);

// ==================== 函数声明 ====================

/**
 * @brief 运动控制模块初始化
 */
void control_init(void);

/**
 * @brief 更新电机实际速度（在中断中调用）
 * @param encoder_pulses 4个电机的本次脉冲数数组
 */
void control_update_speed(const int16_t encoder_pulses[4]);

/**
 * @brief 执行速度闭环控制（在中断中调用）
 * 计算PID输出并设置PWM
 */
void control_run_speed_pid(void);

/**
 * @brief 设置单个电机目标速度
 * @param motor_id 电机ID (0-3)
 * @param speed 目标速度 (脉冲数/10ms，正：前进，负：后退)
 */
void control_set_target_speed(uint8_t motor_id, float speed);

/**
 * @brief 设置所有电机目标速度
 * @param speed1-speed4 四个电机的目标速度
 */
void control_set_all_target_speed(float speed1, float speed2, float speed3, float speed4);

/**
 * @brief 停止所有电机
 */
void control_stop_all(void);

/**
 * @brief 获取电机当前速度
 * @return 当前速度 (脉冲数/10ms)
 */
float control_get_current_speed(uint8_t motor_id);

float get_body_vx(void);

float get_body_vy(void);

// 速度设置和获取接口
void set_target_vx(float vx);
void set_target_vy(float vy);
float get_target_vx(void);
float get_target_vy(void);
float get_body_vx(void);
float get_body_vy(void);


#endif // CONTROL_H
