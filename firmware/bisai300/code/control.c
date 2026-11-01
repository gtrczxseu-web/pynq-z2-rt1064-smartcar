#include "control.h"
#include "main.h"
#include <math.h>

// ==================== 速度PID参数 ====================
// 这些参数需要后期调试确定
#define SPEED_PID_KP                (0.05f)
#define SPEED_PID_KI                (1.0f)
#define SPEED_PID_KD                (0.0f)
#define SPEED_PID_IMAX              (200.0f)
#define SPEED_PID_OUTMAX            (20.0f)
#define MOTOR_PWM_LIMIT              (20.0f)

motor_control_t g_motors[4];

static void motor_set_pwm(uint8_t motor_id, float duty);
static void control_reset_speed_pid(void);

/*
 * 平移停止保持标志：
 * 进入终点稳定阶段后，只允许四轮输出中的纯旋转分量通过，
 * 禁止速度PID残留重新合成平移PWM造成回退。
 */
static uint8_t g_translation_hold = 1;

static void control_reset_speed_pid(void)
{
    for (int i = 0; i < 4; i++) {
        Pid_Param_Init(&g_motors[i].pid,
                       SPEED_PID_KP,
                       SPEED_PID_KI,
                       SPEED_PID_KD,
                       SPEED_PID_IMAX,
                       SPEED_PID_OUTMAX);
        g_motors[i].pwm_duty = 0.0f;
    }
}

/*
 * 每段平移开始前清除上一段速度PID历史，并解除平移输出保持。
 * 该接口由position.c在普通移动和车体系相对移动开始时调用。
 */
void control_begin_translation(void)
{
    control_reset_speed_pid();
    g_translation_hold = 0;
}

/*
 * 进入终点稳定阶段时立即清除速度PID历史。
 * 后续仍允许角度环的纯旋转分量工作，但不允许任何平移PWM输出。
 */
void control_hold_translation(void)
{
    control_reset_speed_pid();
    g_translation_hold = 1;

    for (int i = 0; i < 4; i++) {
        motor_set_pwm(i, 0.0f);
    }
}

//运动控制模块初始化
void control_init(void)
{
    g_translation_hold = 1;

    // 初始化所有电机的控制数据
    for (int i = 0; i < 4; i++) {
        g_motors[i].target_speed = 0.0f;
        g_motors[i].current_speed = 0.0f;
        g_motors[i].pwm_duty = 0.0f;
        g_motors[i].encoder_pulse = 0;
        g_motors[i].encoder_total = 0;

        // 初始化PID参数
        Pid_Param_Init(&g_motors[i].pid,
                       SPEED_PID_KP,
                       SPEED_PID_KI,
                       SPEED_PID_KD,
                       SPEED_PID_IMAX,
                       SPEED_PID_OUTMAX);
    }
}

//执行速度闭环控制（在中断中调用）
void control_update_speed(const int16_t encoder_pulses[4])
{
    for (int i = 0; i < 4; i++) {
        // 保存原始脉冲数
        g_motors[i].encoder_pulse = encoder_pulses[i];

        // 累加总脉冲（可选，用于位置计算）
        g_motors[i].encoder_total += encoder_pulses[i];

        // 直接使用脉冲数作为速度值，单位：脉冲数/20ms
        g_motors[i].current_speed = (float)encoder_pulses[i];
    }
}

void control_run_speed_pid(void)
{
    float error;
    float output;
    float outputs[4];

    for (int i = 0; i < 4; i++) {
        // 计算速度误差 - 直接使用脉冲数差值
        error = g_motors[i].target_speed - g_motors[i].current_speed;

        // 使用增量式PID计算输出
        output = PidIncCtrl(&g_motors[i].pid, error, CONTROL_PERIOD_S);

        // 与PID内部及电机实际输出统一限幅，避免不可执行输出在PID内积累
        outputs[i] = constrain_float(output,
                                     -MOTOR_PWM_LIMIT,
                                     MOTOR_PWM_LIMIT);
    }

    if(g_translation_hold)
    {
        float rotation_output =
            (outputs[0] - outputs[1] +
             outputs[2] - outputs[3]) / 4.0f;

        /*
         * 只保留[+PWM,-PWM,+PWM,-PWM]纯旋转分量。
         * 四轮输出的vx、vy平移分量均为0，因此不会在停车后把车拉回。
         */
        outputs[0] =  rotation_output;
        outputs[1] = -rotation_output;
        outputs[2] =  rotation_output;
        outputs[3] = -rotation_output;
    }

    for (int i = 0; i < 4; i++) {
        // 保存计算出的PWM占空比
        g_motors[i].pwm_duty = outputs[i];

        // 设置电机PWM
        motor_set_pwm(i, outputs[i]);
    }
}

void control_set_target_speed(uint8_t motor_id, float speed)
{
    if (motor_id < 4) {
        g_motors[motor_id].target_speed = speed;
    }
}

void control_set_all_target_speed(float speed1, float speed2, float speed3, float speed4)
{
    float speeds[4] = {speed1, speed2, speed3, speed4};
    for (int i = 0; i < 4; i++) {
        control_set_target_speed(i, speeds[i]);
    }
}

void control_stop_all(void)
{
    g_translation_hold = 1;
    control_reset_speed_pid();

    for (int i = 0; i < 4; i++) {
        control_set_target_speed(i, 0.0f);
        motor_set_pwm(i, 0.0f);
    }
}

float control_get_current_speed(uint8_t motor_id)
{
    if (motor_id < 4) {
        return g_motors[motor_id].current_speed;
    }
    return 0.0f;
}

/**
 * @brief 底层电机PWM设置函数
 * @param motor_id 电机ID (0-3)
 * @param duty 占空比 (-20.0 ~ 20.0)
 *
 * @note PWM占空比限制在20%，符合当前硬件实际输出限制
 */

    //左前轮是1，右前轮是2，左后轮是3，右后轮是4!!!
    // 限幅到 -20.0 ~ 20.0（硬件实际限制）
static void motor_set_pwm(uint8_t motor_id, float duty)
{
    duty = constrain_float(duty, -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);

    //printf("motor%d duty: %.1f\r\n", motor_id, duty);

    switch(motor_id) {
        case 0:  // 左前轮 (电机1)
            if (duty >= 0) {
                gpio_set_level(MOTOR1_DIR, MOTOR1_FORWARD_DIR_LEVEL);
            } else {
                gpio_set_level(MOTOR1_DIR, !MOTOR1_FORWARD_DIR_LEVEL);
            }
            pwm_set_duty(MOTOR1_PWM, (int)(fabs(duty) * (PWM_DUTY_MAX / 100)));
            break;

        case 1:  // 右前轮 (电机2)
            if (duty >= 0) {
                gpio_set_level(MOTOR2_DIR, MOTOR2_FORWARD_DIR_LEVEL);
            } else {
                gpio_set_level(MOTOR2_DIR, !MOTOR2_FORWARD_DIR_LEVEL);
            }
            pwm_set_duty(MOTOR2_PWM, (int)(fabs(duty) * (PWM_DUTY_MAX / 100)));
            break;

        case 2:  // 左后轮 (电机3)
            if (duty >= 0) {
                gpio_set_level(MOTOR3_DIR, MOTOR3_FORWARD_DIR_LEVEL);
            } else {
                gpio_set_level(MOTOR3_DIR, !MOTOR3_FORWARD_DIR_LEVEL);
            }
            pwm_set_duty(MOTOR3_PWM, (int)(fabs(duty) * (PWM_DUTY_MAX / 100)));
            break;

        case 3:  // 右后轮 (电机4)
            if (duty >= 0) {
                gpio_set_level(MOTOR4_DIR, MOTOR4_FORWARD_DIR_LEVEL);
            } else {
                gpio_set_level(MOTOR4_DIR, !MOTOR4_FORWARD_DIR_LEVEL);
            }
            pwm_set_duty(MOTOR4_PWM, (int)(fabs(duty) * (PWM_DUTY_MAX / 100)));
            break;
    }
}

//获取vx的实际值
float get_body_vx(void)
{
    // 从四个电机当前速度解算 vx
    // 逆运动学公式（根据你之前的正运动学推导）：
    // vx = (LF - RF - LB + RB) / 4
    float lf = g_motors[0].current_speed;  // 左前
    float rf = g_motors[1].current_speed;  // 右前
    float lr = g_motors[2].current_speed;  // 左后
    float rr = g_motors[3].current_speed;  // 右后

    return (lf - rf - lr + rr) / 4.0f;
}

//获取vy的实际值
float get_body_vy(void)
{
    float lf = g_motors[0].current_speed;  // 左前
    float rf = g_motors[1].current_speed;  // 右前
    float lr = g_motors[2].current_speed;  // 左后
    float rr = g_motors[3].current_speed;  // 右后

    return (lf + rf + lr + rr) / 4.0f;
}

//获取vx的目标值
float get_target_vx(void)
{
    float lf = g_motors[0].target_speed;  // 左前
    float rf = g_motors[1].target_speed;  // 右前
    float lr = g_motors[2].target_speed;  // 左后
    float rr = g_motors[3].target_speed;  // 右后

    // 逆运动学：vx = (LF - RF - LR + RR) / 4
    return (lf - rf - lr + rr) / 4.0f;
}

//获取vy的目标值
float get_target_vy(void)
{
    float lf = g_motors[0].target_speed;
    float rf = g_motors[1].target_speed;
    float lr = g_motors[2].target_speed;
    float rr = g_motors[3].target_speed;

    // 逆运动学：vy = (LF + RF + LR + RR) / 4
    return (lf + rf + lr + rr) / 4.0f;
}

//利用vx和vy设置四个轮子的目标值
void set_target_v(float vx, float vy)
{
    g_motors[0].target_speed = vy + vx ;   // motor0: 左前轮
    g_motors[1].target_speed = vy - vx ;   // motor1: 右前轮
    g_motors[2].target_speed = vy - vx ;   // motor2: 左后轮
    g_motors[3].target_speed = vy + vx ;   // motor3: 右后轮
}
