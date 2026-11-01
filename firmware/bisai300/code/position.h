#ifndef POSITION_H
#define POSITION_H

#include "zf_common_headfile.h"

// 移动方向宏定义（全局坐标系）
// 注意：这些是期望的移动方向，角度闭环会确保车头大致朝北
#define NORTH   0   // 向北（Y轴正方向）
#define SOUTH   1   // 向南（Y轴负方向）
#define EAST    2   // 向东（X轴正方向）
#define WEST    3   // 向西（X轴负方向）

extern volatile uint8_t g_is_moving;              // 是否正在移动
#define PULSES_PER_GRID_NS  (2430.0f)  // 南北方向一个格子的脉冲数
//原2510
#define PULSES_PER_GRID_EW  (2535.0f)  // 东西方向一个格子的脉冲数

// 函数声明
void position_init(void);
void position_update(void);                 // 20ms中断中调用：更新全局位置
float position_get_x(void);
float position_get_y(void);
void position_reset(float x, float y);      // 用视觉校正，参数单位为两轴各自的脉冲坐标
void position_reset_grid(float grid_x, float grid_y);  // 用格坐标校正
void position_get_grid(float *grid_x, float *grid_y);  // 获取格子坐标

// ========== 新增：定点移动控制 ==========
void position_move(uint8_t direction, float grids);   // 方向 + 格数
void position_stop_move(void);              // 停止移动
uint8_t position_is_moving(void);           // 是否正在移动
void position_set_move_speed(float speed);  // 设置移动速度
void set_target_v(float vx, float vy);
void position_hold_move(uint8_t hold);
void position_get_target_grid(float *grid_x, float *grid_y);
void position_move_to_grid(uint8_t direction, float grid_x, float grid_y);

#endif
