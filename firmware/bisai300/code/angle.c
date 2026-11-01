#include "pynq_telemetry.h"
#include "angle.h"
#include "pid.h"
#include <math.h>

// 角度控制PID参数
#define ANGLE_PID_KP                (2.5f)      // 比例系数，需要调试
#define ANGLE_PID_KI                (0.0f)      // 积分系数
#define ANGLE_PID_KD                (0.0f)      // 微分系数
#define ANGLE_PID_IMAX              (100.0f)    // 积分限幅
#define ANGLE_PID_OUTMAX            (200.0f)    // 输出最大旋转速度

// ==================== 小角度控制参数 ====================
// 实测 vz=10 是目前能够连续旋转的最小指令，角速度约为13deg/s。
// 本版暂不区分平移中纠偏和停车后角度保持，两种场景统一使用以下策略。
#define ANGLE_VZ_CONTINUOUS         (10.0f)     // 最小连续旋转指令
#define ANGLE_STOP_THRESHOLD        (0.20f)     // 进入该误差范围后停止纠偏，单位deg
#define ANGLE_RESTART_THRESHOLD     (0.35f)     // 停止后超过该误差才重新启动，单位deg
#define ANGLE_FINE_RANGE            (1.00f)     // 小角度短脉冲范围，单位deg
#define ANGLE_RATE_DAMPING_KD       (0.15f)     // Z轴角速度阻尼系数

// angle_calc_vz() 每20ms调用一次：
// 输出1个周期(20ms)，等待1个周期(20ms)，再根据新角度决定是否补下一次。
#define ANGLE_FINE_PULSE_ON_CYCLES  (1)
#define ANGLE_FINE_PULSE_OFF_CYCLES (1)

// ==================== IMU量程参数 ====================
#define GYRO_SCALE_FACTOR           (14.3f)     // 原始数据 / 14.3 = deg/s
#define ACC_SCALE_FACTOR            (4098.0f)   // 原始数据 / 4098 = g
#define ANGLE_INTEGRAL_PERIOD       (0.005f)    // 5ms
#define DEG_TO_RAD                  (0.017453292519943295f)
#define RAD_TO_DEG                  (57.29577951308232f)

// ==================== 陀螺仪零漂：填“静止时原始平均值” ====================
// 校正方式固定为：raw - offset。不要在这里再取反。
#define GYRO_ZERO_OFFSET_X          (12.30f)
#define GYRO_ZERO_OFFSET_Y          (-17.43f)
#define GYRO_ZERO_OFFSET_Z          (-7.17f)

// ==================== 方向和轴映射说明 ====================
// 你们实测到的 IMU 原始轴：
//   原始 acc_x 正方向：车头左侧
//   原始 acc_y 正方向：车头方向
//   原始 acc_z 正方向：上方
//   三轴 gyro 在“先减零漂，再取负号”后，满足沿正轴看向负轴逆时针为正。
//
// 四元数 EKF 内部仍采用右手车体系：
//   body_x = 车头方向
//   body_y = 车头左侧
//   body_z = 上方
// 因此读取 IMU 后需要做轴映射：
//   body_x <- raw_y
//   body_y <- raw_x
//   body_z <- raw_z
#define GYRO_BODY_X_FROM_RAW_Y      (1)
#define GYRO_BODY_Y_FROM_RAW_X      (1)
#define GYRO_BODY_Z_FROM_RAW_Z      (1)

// 三轴角速度方向：你们实测为取负号后符合右手正方向
#define GYRO_X_SIGN_AFTER_MAP       (1.0f)     // body_x = +raw_gyro_y
#define GYRO_Y_SIGN_AFTER_MAP       (-1.0f)     // body_y = -raw_gyro_x
#define GYRO_Z_SIGN_AFTER_MAP       (1.0f)     // body_z = +raw_gyro_z，从上往下看逆时针为正

// 加速度方向：raw_y=前、raw_x=左、raw_z=上，暂时不取反
#define ACC_X_SIGN_AFTER_MAP        (-1.0f)      // body_x = -raw_acc_y
#define ACC_Y_SIGN_AFTER_MAP        (1.0f)      // body_y = +raw_acc_x
#define ACC_Z_SIGN_AFTER_MAP        (-1.0f)      // body_z = -raw_acc_z

// ==================== EKF参数 ====================
// Q越大：更相信陀螺预测会变；R越大：越不相信加速度校正。
// 小车运动时加速度计会包含线加速度，所以 R 不要太小。
#define EKF_Q                       (1.0e-5f)
#define EKF_R_ACC                   (2.0e-2f)
#define ACC_NORM_MIN                (0.70f)     // 低于0.7g认为加速度异常，跳过校正
#define ACC_NORM_MAX                (1.30f)     // 高于1.3g认为线加速度较大，跳过校正

// ==================== 启动校准参数 ====================
// 用于解决：开机时角度不是0、静止时gyro零漂导致yaw慢慢偏。
// 注意：angle_init() 必须在 imu660rb 初始化完成后调用。
#define ANGLE_STARTUP_CALIB_SAMPLES (1000)      // 静止采样次数，约1秒
#define ANGLE_STARTUP_DISCARD       (100)       // 丢弃前100次，等待数据稳定


#define GYRO_DEADBAND_DPS           (0.08f)     // 静止小抖动死区，单位 deg/s

#ifndef M_PI
#define M_PI                        (3.14159265358979323846f)
#endif

// ==================== 全局/静态变量 ====================
float g_current_angle = 0.0f;             // 当前 yaw 角，单位：deg，范围 -180~180
static float g_roll_angle = 0.0f;         // 调试用，单位：deg
static float g_pitch_angle = 0.0f;        // 调试用，单位：deg
static float g_yaw_angle = 0.0f;          // 调试用，单位：deg

static pid_param_t g_angle_pid;           // 角度环PID控制器
static float g_target_angle = 0.0f;       // 目标角度
static float g_yaw_rate_dps = 0.0f;       // 当前Z轴角速度，单位deg/s

// 小角度纠偏状态：停止/重启滞回 + 短脉冲节拍
static int g_angle_correction_active = 0;
static int g_fine_pulse_on_count = 0;
static int g_fine_pulse_off_count = 0;
static int g_fine_last_error_sign = 0;

// 动态零漂。启动时会用静止采样重新覆盖，初值为宏定义。
static float g_gyro_zero_x = GYRO_ZERO_OFFSET_X;
static float g_gyro_zero_y = GYRO_ZERO_OFFSET_Y;
static float g_gyro_zero_z = GYRO_ZERO_OFFSET_Z;

// 小车地面航向角单独用Z轴陀螺积分。
// 原因：只有陀螺+加速度时，EKF不能真正观测yaw；加速度只能校正roll/pitch。
static float g_yaw_gyro_angle = 0.0f;

// 四元数状态 q = [q0, q1, q2, q3] = [qw, qx, qy, qz]
static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float P[4][4];                     // 四元数状态协方差

// ==================== 静态函数声明 ====================
static float angle_normalize(float angle);
static void quat_normalize(void);
static void quat_to_euler(void);
static void ekf_init_state(void);
static void angle_startup_calibrate(void);
static void imu_convert_to_body(float *gx_rad, float *gy_rad, float *gz_rad,
                                float *ax_g,   float *ay_g,   float *az_g);
static void ekf_predict(float wx, float wy, float wz, float dt);
static void ekf_update_acc(float ax, float ay, float az);
static int inv3x3(const float A[3][3], float invA[3][3]);

// ==================== 函数实现 ====================

/**
 * @brief 角度模块初始化
 */
void angle_init(void)
{
    g_current_angle = 0.0f;
    g_roll_angle = 0.0f;
    g_pitch_angle = 0.0f;
    g_yaw_angle = 0.0f;
    g_target_angle = 0.0f;
    g_yaw_gyro_angle = 0.0f;
    g_yaw_rate_dps = 0.0f;
    g_angle_correction_active = 0;
    g_fine_pulse_on_count = 0;
    g_fine_pulse_off_count = 0;
    g_fine_last_error_sign = 0;

    ekf_init_state();

    // 开机静止校准。解决开机瞬间角度不是0、启动就大角度偏转的问题。
    // 前提：调用 angle_init() 时小车必须静止，且 imu660rb 已初始化完成。
    angle_startup_calibrate();

    // 初始化角度环PID，保持你原来的结构
    Pid_Param_Init(&g_angle_pid,
                   ANGLE_PID_KP,
                   ANGLE_PID_KI,
                   ANGLE_PID_KD,
                   ANGLE_PID_IMAX,
                   ANGLE_PID_OUTMAX);

    // 保持车头方向
    angle_set_target(0.0f);
}

/**
 * @brief 初始化四元数和协方差
 */
static void ekf_init_state(void)
{
    int i, j;

    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;

    for(i = 0; i < 4; i++) {
        for(j = 0; j < 4; j++) {
            P[i][j] = 0.0f;
        }
    }

    P[0][0] = 1.0e-3f;
    P[1][1] = 1.0e-3f;
    P[2][2] = 1.0e-3f;
    P[3][3] = 1.0e-3f;
}


/**
 * @brief 开机静止校准陀螺零漂，并强制把当前航向设为0度
 *
 * 这个函数解决两个问题：
 * 1. 开机瞬间gyro数据还没稳定，角度环一启动就认为有角度误差；
 * 2. 固定宏零漂不够准，Z轴微小零漂积分后会造成yaw偏移。
 *
 * 注意：调用时小车必须静止，且 IMU 初始化已经完成。
 */
static void angle_startup_calibrate(void)
{
    int i;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    // 丢弃刚上电的前几帧，避免IMU未稳定的数据进入平均值
    for(i = 0; i < ANGLE_STARTUP_DISCARD; i++) {
        imu660rb_get_gyro();
        system_delay_ms(1);
    }

    for(i = 0; i < ANGLE_STARTUP_CALIB_SAMPLES; i++) {
        imu660rb_get_gyro();
        sum_x += (float)imu660rb_gyro_x;
        sum_y += (float)imu660rb_gyro_y;
        sum_z += (float)imu660rb_gyro_z;
        system_delay_ms(1);
    }

    // 这里保存的是“原始gyro静止平均值”，不是取负后的值，也不是除以14.3后的值
    g_gyro_zero_x = sum_x / (float)ANGLE_STARTUP_CALIB_SAMPLES;
    g_gyro_zero_y = sum_y / (float)ANGLE_STARTUP_CALIB_SAMPLES;
    g_gyro_zero_z = sum_z / (float)ANGLE_STARTUP_CALIB_SAMPLES;

    // 开机当前姿态作为0度参考，目标也设为0度
    g_yaw_gyro_angle = 0.0f;
    g_current_angle = 0.0f;
    g_yaw_angle = 0.0f;
    g_target_angle = 0.0f;
    g_yaw_rate_dps = 0.0f;
    g_angle_correction_active = 0;
    g_fine_pulse_on_count = 0;
    g_fine_pulse_off_count = 0;
    g_fine_last_error_sign = 0;

    // 四元数也回到无yaw初值。roll/pitch后续由加速度计慢慢校正。
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
    quat_normalize();
}

/**
 * @brief 角度归一化到 -180~180 度范围
 */
static float angle_normalize(float angle)
{
    while(angle >= 180.0f) angle -= 360.0f;
    while(angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 四元数归一化，避免积分后模长漂移
 */
static void quat_normalize(void)
{
    float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

    if(norm < 1.0e-6f) {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }

    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;
}

/**
 * @brief IMU原始数据转换到 EKF 内部右手车体系
 *
 * EKF内部：body_x=车头，body_y=左侧，body_z=上方。
 * 你们实测：raw_x=左侧，raw_y=车头，raw_z=上方。
 */
static void imu_convert_to_body(float *gx_rad, float *gy_rad, float *gz_rad,
                                float *ax_g,   float *ay_g,   float *az_g)
{
    float raw_gx, raw_gy, raw_gz;

    // 1. 陀螺仪先减零漂。offset填静止原始平均值。
    raw_gx = (float)imu660rb_gyro_x - g_gyro_zero_x;
    raw_gy = (float)imu660rb_gyro_y - g_gyro_zero_y;
    raw_gz = (float)imu660rb_gyro_z - g_gyro_zero_z;

    // 2. 轴映射 + 方向修正 + 单位转换 deg/s -> rad/s
    // body_x = 车头方向 = raw_y；取负后为右手正方向
    *gx_rad = GYRO_X_SIGN_AFTER_MAP * (raw_gy / GYRO_SCALE_FACTOR) * DEG_TO_RAD;

    // body_y = 左侧方向 = raw_x；取负后为右手正方向
    *gy_rad = GYRO_Y_SIGN_AFTER_MAP * (raw_gx / GYRO_SCALE_FACTOR) * DEG_TO_RAD;

    // body_z = 上方方向 = raw_z；取负后从车顶向下看逆时针为正
    *gz_rad = GYRO_Z_SIGN_AFTER_MAP * (raw_gz / GYRO_SCALE_FACTOR) * DEG_TO_RAD;

    // 3. 加速度计轴映射，单位 raw -> g
    // body_x = 车头方向 = raw_acc_y
    *ax_g = ACC_X_SIGN_AFTER_MAP * ((float)imu660rb_acc_y / ACC_SCALE_FACTOR);

    // body_y = 左侧方向 = raw_acc_x
    *ay_g = ACC_Y_SIGN_AFTER_MAP * ((float)imu660rb_acc_x / ACC_SCALE_FACTOR);

    // body_z = 上方方向 = raw_acc_z
    *az_g = ACC_Z_SIGN_AFTER_MAP * ((float)imu660rb_acc_z / ACC_SCALE_FACTOR);
}

/**
 * @brief EKF预测：q(k) = [I + 1/2 Omega dt] q(k-1)
 * @param wx body_x角速度，rad/s
 * @param wy body_y角速度，rad/s
 * @param wz body_z角速度，rad/s
 */
static void ekf_predict(float wx, float wy, float wz, float dt)
{
    int i, j, k;
    float half_dt = 0.5f * dt;
    float A[4][4];
    float q_old[4];
    float AP[4][4];
    float P_new[4][4];

    // A = I + 1/2 * Omega * dt，对应 q_dot = 1/2 * omega_q * q
    A[0][0] = 1.0f;          A[0][1] = -wx * half_dt; A[0][2] = -wy * half_dt; A[0][3] = -wz * half_dt;
    A[1][0] = wx * half_dt;  A[1][1] = 1.0f;          A[1][2] = -wz * half_dt; A[1][3] = wy * half_dt;
    A[2][0] = wy * half_dt;  A[2][1] = wz * half_dt;  A[2][2] = 1.0f;          A[2][3] = -wx * half_dt;
    A[3][0] = wz * half_dt;  A[3][1] = -wy * half_dt; A[3][2] = wx * half_dt;  A[3][3] = 1.0f;

    for(i = 0; i < 4; i++) {
        q_old[i] = q[i];
    }

    for(i = 0; i < 4; i++) {
        q[i] = 0.0f;
        for(j = 0; j < 4; j++) {
            q[i] += A[i][j] * q_old[j];
        }
    }
    quat_normalize();

    // P = A * P * A^T + Q
    for(i = 0; i < 4; i++) {
        for(j = 0; j < 4; j++) {
            AP[i][j] = 0.0f;
            for(k = 0; k < 4; k++) {
                AP[i][j] += A[i][k] * P[k][j];
            }
        }
    }

    for(i = 0; i < 4; i++) {
        for(j = 0; j < 4; j++) {
            P_new[i][j] = 0.0f;
            for(k = 0; k < 4; k++) {
                P_new[i][j] += AP[i][k] * A[j][k];   // A^T[k][j] = A[j][k]
            }
        }
    }

    for(i = 0; i < 4; i++) {
        for(j = 0; j < 4; j++) {
            P[i][j] = P_new[i][j];
        }
        P[i][i] += EKF_Q;
    }
}

/**
 * @brief 3x3矩阵求逆
 * @return 1成功，0失败
 */
static int inv3x3(const float A[3][3], float invA[3][3])
{
    float det;

    det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
        - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
        + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    if(det > -1.0e-9f && det < 1.0e-9f) {
        return 0;
    }

    invA[0][0] =  (A[1][1] * A[2][2] - A[1][2] * A[2][1]) / det;
    invA[0][1] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]) / det;
    invA[0][2] =  (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det;

    invA[1][0] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]) / det;
    invA[1][1] =  (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det;
    invA[1][2] = -(A[0][0] * A[1][2] - A[0][2] * A[1][0]) / det;

    invA[2][0] =  (A[1][0] * A[2][1] - A[1][1] * A[2][0]) / det;
    invA[2][1] = -(A[0][0] * A[2][1] - A[0][1] * A[2][0]) / det;
    invA[2][2] =  (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / det;

    return 1;
}

/**
 * @brief EKF加速度计更新：用归一化加速度方向校正roll/pitch
 *
 * 注意：只有加速度计无法长期校正 yaw 漂移，yaw 仍主要来自 gyro_z 积分。
 */
static void ekf_update_acc(float ax, float ay, float az)
{
    int i, j, k;
    float norm;
    float z[3];
    float h[3];
    float H[3][4];
    float PHt[4][3];
    float S[3][3];
    float invS[3][3];
    float K[4][3];
    float residual[3];
    float KH[4][4];
    float I_KH[4][4];
    float P_new[4][4];
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    // 加速度归一化，异常时跳过观测更新
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if(norm < ACC_NORM_MIN || norm > ACC_NORM_MAX) {
        return;
    }

    z[0] = ax / norm;
    z[1] = ay / norm;
    z[2] = az / norm;

    // 水平静止时，body加速度方向约为 [0, 0, +1]
    // h(q) = R(q) * [0, 0, 1]^T
    h[0] = 2.0f * (q1 * q3 + q0 * q2);
    h[1] = 2.0f * (q2 * q3 - q0 * q1);
    h[2] = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

    // H = dh/dq，q顺序为 [q0,q1,q2,q3]
    H[0][0] =  2.0f * q2;  H[0][1] =  2.0f * q3;  H[0][2] =  2.0f * q0;  H[0][3] =  2.0f * q1;
    H[1][0] = -2.0f * q1;  H[1][1] = -2.0f * q0;  H[1][2] =  2.0f * q3;  H[1][3] =  2.0f * q2;
    H[2][0] =  0.0f;       H[2][1] = -4.0f * q1;  H[2][2] = -4.0f * q2;  H[2][3] =  0.0f;

    residual[0] = z[0] - h[0];
    residual[1] = z[1] - h[1];
    residual[2] = z[2] - h[2];

    // PHt = P * H^T，尺寸 4x3
    for(i = 0; i < 4; i++) {
        for(j = 0; j < 3; j++) {
            PHt[i][j] = 0.0f;
            for(k = 0; k < 4; k++) {
                PHt[i][j] += P[i][k] * H[j][k];
            }
        }
    }

    // S = H * P * H^T + R，尺寸 3x3
    for(i = 0; i < 3; i++) {
        for(j = 0; j < 3; j++) {
            S[i][j] = 0.0f;
            for(k = 0; k < 4; k++) {
                S[i][j] += H[i][k] * PHt[k][j];
            }
        }
        S[i][i] += EKF_R_ACC;
    }

    if(!inv3x3(S, invS)) {
        return;
    }

    // K = PHt * invS，尺寸 4x3
    for(i = 0; i < 4; i++) {
        for(j = 0; j < 3; j++) {
            K[i][j] = 0.0f;
            for(k = 0; k < 3; k++) {
                K[i][j] += PHt[i][k] * invS[k][j];
            }
        }
    }

    // q = q + K * residual
    for(i = 0; i < 4; i++) {
        for(j = 0; j < 3; j++) {
            q[i] += K[i][j] * residual[j];
        }
    }
    quat_normalize();

    // P = (I - K H) P
    for(i = 0; i < 4; i++) {
        for(j = 0; j < 4; j++) {
            KH[i][j] = 0.0f;
            for(k = 0; k < 3; k++) {
                KH[i][j] += K[i][k] * H[k][j];
            }
            I_KH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
        }
    }

    for(i = 0; i < 4; i++) {
        for(j = 0; j < 4; j++) {
            P_new[i][j] = 0.0f;
            for(k = 0; k < 4; k++) {
                P_new[i][j] += I_KH[i][k] * P[k][j];
            }
        }
    }

    for(i = 0; i < 4; i++) {
        for(j = 0; j < 4; j++) {
            P[i][j] = P_new[i][j];
        }
    }
}

/**
 * @brief 四元数转欧拉角，单位deg
 */
static void quat_to_euler(void)
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float sinp;

    g_roll_angle = atan2f(2.0f * (q0 * q1 + q2 * q3),
                          1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;

    sinp = 2.0f * (q0 * q2 - q3 * q1);
    if(sinp > 1.0f) sinp = 1.0f;
    if(sinp < -1.0f) sinp = -1.0f;
    g_pitch_angle = asinf(sinp) * RAD_TO_DEG;

    g_yaw_angle = atan2f(2.0f * (q0 * q3 + q1 * q2),
                         1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;

    // 注意：g_current_angle 不再直接取四元数yaw。
    // 小车地面航向使用 g_yaw_gyro_angle，避免加速度EKF更新影响yaw。
}

/**
 * @brief 更新当前角度（在5ms中断中调用）
 */
void angle_update(void)
{
    float gx_rad, gy_rad, gz_rad;
    float ax_g, ay_g, az_g;

    // 获取IMU原始数据，保留你原来的调用位置
    imu660rb_get_gyro();
    pynq_raw_gyro(imu660rb_gyro_z);
    imu660rb_get_acc();
    

    // 原始IMU数据 -> EKF内部右手车体系
    imu_convert_to_body(&gx_rad, &gy_rad, &gz_rad, &ax_g, &ay_g, &az_g);

    // 小车平面航向角：单独用映射后的 body_z 角速度积分。
    // 不用四元数yaw直接作为闭环角度，因为加速度计无法观测yaw，容易出现
    // “车已经偏了，但EKF的yaw仍被拉回0附近”的假象。
    {
        float gz_dps = gz_rad * RAD_TO_DEG;
        if(gz_dps > -GYRO_DEADBAND_DPS && gz_dps < GYRO_DEADBAND_DPS) {
            gz_dps = 0.0f;
        }
        g_yaw_rate_dps = gz_dps;
        g_yaw_gyro_angle = angle_normalize(g_yaw_gyro_angle + gz_dps * ANGLE_INTEGRAL_PERIOD);
        g_current_angle = g_yaw_gyro_angle;
    }

    // 四元数EKF仍然运行，用于roll/pitch调试和姿态稳定；但不直接覆盖g_current_angle。
    ekf_predict(gx_rad, gy_rad, gz_rad, ANGLE_INTEGRAL_PERIOD);
    ekf_update_acc(ax_g, ay_g, az_g);
    quat_to_euler();

    // 调试：data[0]闭环使用角度；data[1]映射后Z轴角速度deg/s；data[2]四元数yaw，仅作参考
    /*
    seekfree_assistant_oscilloscope_data.data[0] = g_current_angle;
    seekfree_assistant_oscilloscope_data.data[1] = gz_rad * RAD_TO_DEG;
    seekfree_assistant_oscilloscope_data.data[2] = g_yaw_angle;
    seekfree_assistant_oscilloscope_data.channel_num = 3;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
    seekfree_assistant_data_analysis();
*/
}

/**
 * @brief 获取当前角度 yaw
 */
float angle_get_current(void)
{
    return g_current_angle;
}

float angle_get_roll(void)
{
    return g_roll_angle;
}

float angle_get_pitch(void)
{
    return g_pitch_angle;
}

float angle_get_yaw(void)
{
    return g_yaw_angle;
}

/**
 * @brief 设置目标角度
 */
void angle_set_target(float target_angle)
{
    float new_target = angle_normalize(target_angle);
    float target_change = angle_normalize(new_target - g_target_angle);

    g_target_angle = new_target;

    // 只有目标实际变化时才重置状态，避免上层重复设置同一目标后
    // 破坏20ms输出、20ms等待的短脉冲节拍。
    if(fabsf(target_change) > 1.0e-4f) {
        g_angle_correction_active = 0;
        g_fine_pulse_on_count = 0;
        g_fine_pulse_off_count = 0;
        g_fine_last_error_sign = 0;
    }
}

/**
 * @brief 手动重置当前yaw角；例如起点、视觉重新定位、停车校准后调用
 */
void angle_reset_current(float current_angle)
{
    float half_yaw = current_angle * DEG_TO_RAD * 0.5f;

    g_yaw_gyro_angle = angle_normalize(current_angle);
    g_current_angle = g_yaw_gyro_angle;
    g_yaw_rate_dps = 0.0f;
    g_angle_correction_active = 0;
    g_fine_pulse_on_count = 0;
    g_fine_pulse_off_count = 0;
    g_fine_last_error_sign = 0;

    q[0] = cosf(half_yaw);
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = sinf(half_yaw);
    quat_normalize();
    quat_to_euler();
}

/**
 * @brief 计算纠偏所需的旋转速度 vz（在20ms中断中调用）
 * @return 旋转速度，保持你原来的符号逻辑
 */
float angle_calc_vz(void)
{
    float error = -g_target_angle + g_current_angle;
    float abs_error;
    float vz;
    int error_sign;

    if(error > 180.0f) {
        error -= 360.0f;
    } else if(error < -180.0f) {
        error += 360.0f;
    }

    abs_error = fabsf(error);

    // 已经进入停止范围：关闭纠偏，并清除尚未执行的短脉冲。
    if(abs_error <= ANGLE_STOP_THRESHOLD) {
        g_angle_correction_active = 0;
        g_fine_pulse_on_count = 0;
        g_fine_pulse_off_count = 0;
        g_fine_last_error_sign = 0;
        return 0.0f;
    }

    // 停止后使用更大的重启阈值，防止0.20~0.35deg附近反复启停。
    if(!g_angle_correction_active) {
        if(abs_error < ANGLE_RESTART_THRESHOLD) {
            return 0.0f;
        }

        g_angle_correction_active = 1;
        g_fine_pulse_on_count = 0;
        g_fine_pulse_off_count = 0;
        g_fine_last_error_sign = 0;
    }

    error_sign = (error > 0.0f) ? 1 : -1;

    // 1deg以内不连续输出最低转速，改为20ms短脉冲、20ms等待、重新测量。
    if(abs_error <= ANGLE_FINE_RANGE) {
        // 如果误差已经越过目标改变符号，立即取消上一方向剩余的脉冲/等待。
        if(error_sign != g_fine_last_error_sign) {
            g_fine_pulse_on_count = 0;
            g_fine_pulse_off_count = 0;
            g_fine_last_error_sign = error_sign;
        }

        if(g_fine_pulse_on_count > 0) {
            g_fine_pulse_on_count--;
            if(g_fine_pulse_on_count == 0) {
                g_fine_pulse_off_count = ANGLE_FINE_PULSE_OFF_CYCLES;
            }
            return (float)error_sign * ANGLE_VZ_CONTINUOUS;
        }

        if(g_fine_pulse_off_count > 0) {
            g_fine_pulse_off_count--;
            return 0.0f;
        }

        g_fine_pulse_on_count = ANGLE_FINE_PULSE_ON_CYCLES;
        g_fine_pulse_on_count--;
        if(g_fine_pulse_on_count == 0) {
            g_fine_pulse_off_count = ANGLE_FINE_PULSE_OFF_CYCLES;
        }
        return (float)error_sign * ANGLE_VZ_CONTINUOUS;
    }

    // 离开精调区后清除短脉冲状态，恢复连续角度控制。
    g_fine_pulse_on_count = 0;
    g_fine_pulse_off_count = 0;
    g_fine_last_error_sign = 0;

    // 原比例控制叠加Z轴角速度阻尼。
    // 当前误差定义为 current-target，原有控制符号保持不变；
    // 纠偏时角速度通常与误差符号相反，因此同号叠加可降低接近目标时的输出。
    vz = PidLocCtrl(&g_angle_pid, error, 0.02f)
       + ANGLE_RATE_DAMPING_KD * g_yaw_rate_dps;

    // 连续纠偏时避开vz=1~9的不连续爬行区。
    if(vz > 0.0f && vz < ANGLE_VZ_CONTINUOUS) {
        vz = ANGLE_VZ_CONTINUOUS;
    } else if(vz < 0.0f && vz > -ANGLE_VZ_CONTINUOUS) {
        vz = -ANGLE_VZ_CONTINUOUS;
    } else if(vz == 0.0f) {
        vz = (float)error_sign * ANGLE_VZ_CONTINUOUS;
    }

    vz = constrain_float(vz, -ANGLE_PID_OUTMAX, ANGLE_PID_OUTMAX);

    return vz;
}
