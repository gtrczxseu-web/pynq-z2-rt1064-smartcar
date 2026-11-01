#ifndef ANGLE_H
#define ANGLE_H

#include "zf_common_headfile.h"

// 角度解算相关
// 注意：不要在 .h 里写 static float g_current_angle，否则每个 .c 都会有一份自己的变量。
extern float g_current_angle;    // 当前 yaw 角，单位 deg，范围 -180~180

// 函数声明
void angle_init(void);                          // 角度模块初始化
void angle_update(void);                        // 5ms中断中调用：更新当前角度
float angle_get_current(void);                  // 获取当前 yaw 角
void angle_set_target(float target_angle);      // 设置目标角度
float angle_calc_vz(void);                      // 计算纠偏所需的旋转速度 vz

// 调试/校准用
float angle_get_roll(void);
float angle_get_pitch(void);
float angle_get_yaw(void);
void angle_reset_current(float current_angle);

static void rotate_ccw_90(void);
static void rotate_cw_90(void);
static void rotate_180(void);

#endif
