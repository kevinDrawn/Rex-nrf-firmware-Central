/*
 * motor_control.h
 *
 *  Created on: Nov 20, 2024
 *      Author: qoxmf
 */
#pragma once

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include <zephyr/sys/atomic.h>
#include <float.h>
#include <stdbool.h>
#include <zephyr/sys/printk.h>

#include <filter_central.h>
#include <can.h>
#include <uart_bridge.h>
#include <esb_rx.h>
#include <torque_cyl.h>
#include <adc_sampler.h>
#include <trajectory_library.h>

#define SAMPLING_INTERVAL 0.001f 
#define TICKS_PER_DEGREE (4096.0f / 360.0f)
#define TICKS_PER_RADIAN (4096.0f / (2.0f * M_PI))
#define TICKS_TO_RADIAN ((2.0f * M_PI) / 4096.0f)
#define RAD_PER_SEC_TO_RPM (60.0f / (2.0f * M_PI))  // ≈ 9.5493f
#define RPM_TO_RAD_PER_SEC (2.0f * M_PI / 60.0f)

#define MOTOR_COMMAND_RATIO (100.0f / 0.263f) // [0.263 N.m - 100.0 %]
#define Pulley_Radius 12 // mm

// Motor Hardware Limit
#define MAXIMUM_VEL_RPM 250.0f //RPM
#define MAXIMUM_VEL (MAXIMUM_VEL_RPM * RPM_TO_RAD_PER_SEC) // rad/s
#define MAXIMUM_ACC_RPM 3000.0f // RPM/s
#define MAXIMUM_ACC (MAXIMUM_ACC_RPM * RPM_TO_RAD_PER_SEC) // rad/s^2

// Nominal Boundary
#define MAX_ACC_NOMINAL 300 // rad/s^2
#define theta_min_axial deg2rad(50)
#define theta_max_axial deg2rad(130)

// Acceleration Boundary Algorithm
#define at 0.01f

#if defined(CONFIG_HALL_MODE) && defined(CONFIG_CAN_MODE)
#define LOG_CHANNEL 6
#elif defined(CONFIG_HALL_MODE)
#define LOG_CHANNEL 4
#endif

/* Motor Controller Parameter */
// Position Boundary Limitation
#define qi 100	// ticks Fix - Soft margin
#define qs 0		// ticks Fix - 완충구간
#define xi 300 	// RPM 감속 시작점에서 허용되는 최대 속도 제한, Damping Coefficient
#define torque_clamp 0.000f

#if defined(CONFIG_CONTRALATERAL_MODE)

// Passivity Admittance Gain 
#define Kp_adm 10
#define K_adm 2 // 2
#define mirror_bias deg2rad(0)

/* Robust Gain */
#define	Kp 1500
#define	Kv 30

// /* Compliance Gain */
// #define	Kp 2500
// #define	Kv 15

extern float Nominal_I;
extern float Nominal_C;
extern float Nominal_K;

#elif defined(CONFIG_FIXED_MODE)
#define Kp_fixed 800
#define Kd_fixed 10
#endif

/* Oscillate 방지용 파라미터 */
// ===== Timing =====
#define CONTROL_DT_MS          1
#define OSC_WINDOW_MS          100
#define OSC_WINDOW_SIZE        (OSC_WINDOW_MS / CONTROL_DT_MS)

// 반측 운동성 검증 Window
#define CONTRA_WIN_MS     500
#define CONTRA_WIN_SIZE   (CONTRA_WIN_MS / CONTROL_DT_MS)

// ===== Power & Zero Crossing =====
#define POWER_THRESHOLD        10.0f     // |power| threshold
#define ZC_THRESHOLD           5          // zero crossing count
#define ZC_ENTER  4

// ===== Contra ankle activity =====
#define QDOT_R_QUIET_TH   0.3 // radian  
#define QDOT_R_ACTIVE_TH  1 // radian   

#define ZC_ENTER_MIN   3
#define ZC_ENTER_MAX   5   
#define ZC_EXIT   0

// Ring Buffer scale
#define buffer_scale 100 // radian * 100

// Admittance Controller Validation
#define MAX_ANGLE_DEG 25.0f
#define AMPLITUDE_TICKS (MAX_ANGLE_DEG * TICKS_PER_DEGREE)
#define FREQUENCY_HZ 1.0f
#define SAMPLE_TIME_MS 1000       // 1kHz

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

// Admittance Controller
#define spring_zero 135*M_PI/180

/* Mirror Controller */
// #define MIN_PERIOD_MS 100 // 300 ms -> 0.3 s Right Maximum Cycle Period
#define MIN_PERIOD_MS 100 // 300 ms -> 0.3 s Right Maximum Cycle Period
#define MAX_PERIOD_MS 1500 // 750 ms -> 0.75 s Left Maximum Cycle Period

// #define MIN_OFFSET_MS 100 // 300 ms -> 0.3 s Right Maximum Cycle Period
#define MIN_OFFSET_MS 100 // 300 ms -> 0.3 s Right Maximum Cycle Period
#define MAX_OFFSET_MS 750 // 750 ms -> 0.75 s Left Maximum Cycle Period

#define ANG_BUF_CAP   1024
#define PERIOD_MED_K  1

// Duplicate Prevention Parameter
#define EMA_ALPHA	  0.1f //각도 EMA 필터 계수 (0.05 ~ 0.2 recommended)

// Hysterisis (rad/s) — 이전에 10 deg/s였다면 10*π/180 ≈ 0.1745 rad/s
#define VEL_HYST_RAD_S         0.15f 	// if noise big, increas, if lose peak, reduce
#define REFRACT_MS             100				 // Refractory Period - Ignore next peak detact before refractory period
#define OFFSET_SLEW_MS 		   1  			     // Offset update interval
#define PEAK_DIFF_THRESH    deg2rad(10.0f)   // 직전 Peak(+ -)과 이번에 감지된 Peak(+ -) 사이 높이 제한 threshold 인접한 peak의 작은 봉우리 제거 목적

#define offset_regression_a    0.43f
// #define offset_regression_b	   0.00f // Sec
#define offset_regression_b	   -0.05f

// Sine Trajectory
#define ADC_MIN   0.0f
#define ADC_MAX   4096.0f
#define F_MIN     1.0f     // 최소 1 Hz
#define F_MAX     3.0f     // 최대 3 Hz
#define F_STEP    0.1f    // 주파수 단위 간격
#define DF_MAX_PER_SEC  2.0f   // 주파수 변화 최대 속도 (Hz/s)
#define TWO_PI    6.28318530718f
#define SINE_BIAS 70

// Logging Parameter
#define LOG_BATCH_EXO      10   // 1 kHz * 10 ms
#define LOG_BATCH_EMG      20   // 2 kHz * 10 ms

#define LOG_EXO_CHANNEL    6    // qL, qR, qMirror, torque_axis, torque_can_L, torque_can_R
#define LOG_EMG_CHANNEL    4    // L_TB, L_GT, R_TB, R_GT

#if defined(CONFIG_CONTRALATERAL_MODE)
// Gait Trajectory
#define f_target 1
#elif defined(CONFIG_FIXED_MODE)
#define f_target 2
#endif

typedef struct
{
    /* ============================================
     * Initialize
     * ============================================ */
    bool init_done;

    /* ============================================
     * Joint Space Variables
     * ============================================ */
    float q_L, q_R, q_mirror, q_fixed;
    float q_prev_L, q_prev_R, q_prev_mirror, q_prev_q_fixed;
    float q_dot_L, q_dot_R, q_dot_mirror, q_dot_fixed;

    /* ============================================
     * EMG Variables
     * ============================================ */
    int32_t L_TB_1, L_TB_2, L_GT_1, L_GT_2;
    int32_t R_TB_1, R_TB_2, R_GT_1, R_GT_2;

    /* ============================================
     * Peak Detection (Right Joint)
     * ============================================ */
    bool was_vel_pos_r;
    bool was_vel_neg_r;

    uint32_t last_peak_r_ms;
    uint32_t last_peak_ms;

    float last_peak_q;
    uint8_t peak_valid;
    uint8_t peak_flag;

    /* Trough (옵션 – 지금은 미사용) */
    float last_trough_q;
    bool have_trough;

    /* ============================================
     * Gait Period Estimation
     * ============================================ */
    uint32_t refractory_util_r;
    uint32_t last_period_r_ms;
    uint32_t period_ms;

    uint16_t med_ms;
    uint8_t event_trigger;

    /* ============================================
     * Torque Variables
     * ============================================ */
    float torque_axis;
    float torque_ext_clamp;
    float torque_admittance;
    float torque_can_left;
    float torque_can_right;
    
    /* ============================================
     * Nominal Plant State
     * q, q_dot, q_ddot
     * ============================================ */
    float nominal_q_ddot;
    float nominal_q_dot;
    float nominal_q;

    /* ============================================
     * Mirror Offset Buffer
     * q history buffer for time-delay mirror gait
     * ============================================ */
    int16_t angle_buffer[ANG_BUF_CAP];

    volatile uint16_t head;     // next write index
    volatile uint32_t filled;   // number of valid entries

    /* Offset (ms) tracking */
    float offset_sec;
    volatile uint16_t offset_ms;        // current offset
    volatile uint16_t target_offset_ms; // target offset

    /* Period median buffer */
    uint32_t period_buf[PERIOD_MED_K];
    uint8_t  period_idx;
    uint8_t  period_count;

    /* ============================================
     * Motor Output Command
     * ============================================ */
    int32_t command_vel;

    /* ============================================
     * Motor Position Boundary
     * ============================================ */
    int32_t max_limit;
    int32_t min_limit;

    int32_t ticks_diff;
    int32_t initial_ticks_adm;
    float prev_ticks_diff;

    int32_t time;
    int32_t time_start;

    Filter torque_axis_filter;
    Filter motor_command_filter;
    Filter dot_filter_L;
    Filter dot_filter_R;

    Filter filter_L;
    Filter filter_R;

    Filter filter_L_can;
    Filter filter_R_can;
    Filter filter_scale;
} motor_control_state_t;

extern motor_control_state_t mc;

/* 모터 */
extern int8_t admittance_motor;

/* End Mirror Controller */
extern int8_t start_flag;
extern int8_t stop_flag;
extern uint8_t stop_flag_upper[2];
extern uint8_t stop_flag_lower[2];

extern const int16_t MAX_POSITION_LIMIT[2];
extern const int16_t MIN_POSITION_LIMIT[2];

extern float interpolation_coefficients[50][4];
extern const float normal_ankleAngles[100];

// Position Limit - Exo geometric parameter
extern float b[2];
extern float p[2];

extern const float b_can[2];
extern const float p_can[2];

extern const float b_ax[2];
extern const float p_ax[2];

// Sine 궤적 생성
extern float f_cur;    // 현재 주파수 (Hz)
extern float phase;    // 현재 위상 (rad)

extern float limit_unwind_ticks_An_left, limit_wind_ticks_An_left;
extern float limit_unwind_ticks_An_right, limit_wind_ticks_An_right;

// Controller Init
void control_init(void);
void control_filter_init(void);

// UART Test
void UART_Test_step(void);

// Control Cycle
void Mirror_complinace_control_step(const struct device *can_dev);

// Mirror Controller
static inline float compute_offset_sec(float period_sec);

// Position Limit
float L_Norm(float An_deg, uint8_t exo_type);
void Position_limit_torque_control(float An_rad, uint8_t exo_type);

#endif /* INC_MOTOR_CONTROL_H_ */
