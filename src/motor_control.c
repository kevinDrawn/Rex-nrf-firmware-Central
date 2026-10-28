/*
 * motor_control.c
 *
 *  Created on: Nov 20, 2024
 *      Author: qoxmf
 */

#include "motor_control.h"

LOG_MODULE_REGISTER(motor_control, LOG_LEVEL_INF);

// Logging
volatile uint8_t tx_busy = 0;

int8_t start_flag = 0;
int8_t stop_flag = 0;

uint8_t stop_flag_upper[2] = {0, 0};
uint8_t stop_flag_lower[2] = {0, 0};

const int16_t MAX_POSITION_LIMIT[2] =  {160, 570};
const int16_t MIN_POSITION_LIMIT[2] =  {-400, -120};

// Control step initial stabilization
static bool control_initialized = false;
static const uint32_t INIT_DELAY_MS = 3000;

const float normal_ankleAngles[100] = {
    92.21352322, 91.25561067, 89.94946992, 88.32863933, 86.44527565, 84.36778162, 82.17698221, 79.96108329,
    77.80978766, 75.80805555, 74.03006512, 72.53394116, 71.35777101, 70.51731854, 70.0056879, 69.79499488,
    69.83989714, 70.08263755, 70.45909276, 70.90521034, 71.36317741, 71.78669721, 72.14485333, 72.42420353,
    72.62894509, 72.77920876, 72.90774117, 73.05540333, 73.26602559, 73.5812064, 74.03561953, 74.65330896,
    75.4453157, 76.40881477, 77.5277664, 78.77492289, 80.11490086, 81.50793896, 82.91392039, 84.29624637,
    85.62519455, 86.88047394, 88.05278352, 89.14428101, 90.16796228, 91.14603219, 92.10741081, 93.08456557,
    94.10989095, 95.21187723, 96.41132135, 97.71783771, 99.12692466, 100.6178309, 102.1524418, 103.6753643,
    105.1153281, 106.3879392, 107.399724, 108.0532931, 108.2533429, 107.9131171, 106.9608734, 105.3458643,
    103.0433439, 100.0581662, 96.42663563, 92.21640373, 87.52436446, 82.47266225, 77.20308413, 71.87023417,
    66.63397817, 61.65168799, 57.0708067, 53.02220104, 49.61467596, 46.93090904, 45.02493567, 43.92119327,
    43.61502603, 44.07447052, 45.24309081, 47.04360867, 49.38207762, 52.15237019, 55.24077939, 58.53056897,
    61.90633715, 65.25808063, 68.48485948, 71.49796998, 74.22353624, 76.60443635, 78.60148958, 80.19385195,
    81.37859764, 82.16950488, 82.59511111, 90
};

// Position Limit - Cable & Sheath end position
float b[2] = {-11.0, 20.38}; // mm
float p[2] = {14.0, 5.43};  // mm

float Nominal_I = 2 * (1e-6f);
float Nominal_C = 2 * (1e-4f);
float Nominal_K = 10 * (1e-3f);

float limit_unwind_ticks_An_right = 0.0f;
float limit_wind_ticks_An_right = 0.0f;
float limit_unwind_ticks_An_left = 0.0f;
float limit_wind_ticks_An_left = 0.0f;

float f_cur = 0.0f;   // 초기값 원하는 대로
float phase = 0.0f;

float x_increment = 0.0;
float x = 0.0;
float Gait_output = 0.0;

// Logging Buffer
static int16_t log_qL[LOG_BATCH_EXO];
static int16_t log_qR[LOG_BATCH_EXO];
static int16_t log_qMirror[LOG_BATCH_EXO];
static int16_t log_torque[LOG_BATCH_EXO];
static int16_t log_torque_can_L[LOG_BATCH_EXO];
static int16_t log_torque_can_R[LOG_BATCH_EXO];

static int32_t log_tb_emg[LOG_BATCH_EMG];
static int32_t log_gt_emg[LOG_BATCH_EMG];
static int32_t log_r_tb_emg[LOG_BATCH_EMG];
static int32_t log_r_gt_emg[LOG_BATCH_EMG];

/* Keep full signed samples in RAM; serialize only their lower 24 bits. */
static void put_emg_i24_be(uint8_t *dst, const int32_t *samples)
{
    for (size_t i = 0; i < LOG_BATCH_EMG; ++i) {
        uint32_t value = (uint32_t)samples[i];
        dst[0] = (uint8_t)(value >> 16);
        dst[1] = (uint8_t)(value >> 8);
        dst[2] = (uint8_t)value;
        dst += EMG_WIRE_SAMPLE_BYTES;
    }
}

static uint8_t exo_sample_count = 0;
static uint8_t emg_sample_count = 0;

static float power_buffer[OSC_WINDOW_SIZE];
static int   power_sign_buffer[OSC_WINDOW_SIZE];

static int buffer_idx = 0;
static int zc_count = 0;

static bool oscillation_mode = false;
static int last_sign = 0;

float qdotR_buf[CONTRA_WIN_SIZE];
int   qdotR_idx = 0;

motor_control_state_t mc = {

	/* ============================================
     * Initialize
     * ============================================ */
	.init_done = false,

	/* ============================================
     * Joint Variables
     * ============================================ */
    .q_L = 0.0f,
    .q_R = 0.0f,
    .q_fixed = 0.0f,
    .q_mirror = 0.0f,
    .q_prev_L = 0.0f,
    .q_prev_R = 0.0f,
    .q_prev_mirror = 0.0f,
    .q_prev_q_fixed = 0.0f,
    .q_dot_L = 0.0f,
    .q_dot_R = 0.0f,
    .q_dot_fixed = 0.0f,
	.q_dot_mirror = 0.0f,

	/* ============================================
     * EMG Variables
     * ============================================ */
    .L_TB_1 = 0.0f,
    .L_TB_2 = 0.0f,
    .L_GT_1 = 0.0f,
    .L_GT_2 = 0.0f,
    .R_TB_1 = 0,
    .R_GT_1 = 0,
    .R_TB_2 = 0,
    .R_GT_2 = 0,

    /* ============================================
     * Peak Detection
     * ============================================ */
    .was_vel_pos_r = false,
    .was_vel_neg_r = false,
    .last_peak_r_ms = 0,
    .last_peak_ms = 0,
    .last_peak_q = 0.0f,
    .peak_valid = 0,
	.peak_flag = 0,

    /* Optional trough tracking */
    .last_trough_q = 0.0f,
    .have_trough = false,

    /* ============================================
     * Gait Period Estimation
     * ============================================ */
    .refractory_util_r = 0,
    .last_period_r_ms = 0,
    .period_ms = 0,
    .med_ms = 220,     // 너가 썼던 초기값 적용
    .event_trigger = 0,

    /* ============================================
     * Torque
     * ============================================ */
    .torque_axis = 0.0f,
    .torque_can_left = 0.0f,
    .torque_can_right = 0.0f,
    
    .torque_ext_clamp = 0.0f,
    .torque_admittance = 0.0f,

    /* ============================================
     * Nominal Plant State
     * ============================================ */
    .nominal_q_ddot = 0.0f,
    .nominal_q_dot = 0.0f,
    .nominal_q = 0.0f,

    /* ============================================
     * Mirror Offset Buffer
     * ============================================ */
    .angle_buffer = {0},   // 전체 0 초기화

    .head = 0,
    .filled = 0,

	.offset_sec = 0,
    .offset_ms = 220,
    .target_offset_ms = 220,

    .period_buf = {0},
    .period_idx = 0,
    .period_count = 0,

    /* ============================================
     * Motor Command
     * ============================================ */
    .command_vel = 0,

	/* ============================================
     * Motor Position Boundary
     * ============================================ */
	.max_limit = 0,
    .min_limit = 0,
	.ticks_diff = 0,
	.initial_ticks_adm = 0,
	.prev_ticks_diff = 0.0,

	/* ============================================
     * Timer
     * ============================================ */
	.time = 0,
    .time_start = 0,
};

uint32_t last_log_ms = 0;

void control_filter_init(void){

    // Filter 등록
	Filter_Init(&mc.torque_axis_filter, cutoff_freqs_torque_axis, 20);
	Filter_Init(&mc.motor_command_filter, cutoff_freqs_motor_command, 20);

	Filter_Init(&mc.filter_L_can, cutoff_freqs_torque_axis, 20);
	Filter_Init(&mc.filter_R_can, cutoff_freqs_torque_axis, 20);

	Filter_Init(&mc.filter_L, cutoff_freqs_hall, 20);
	Filter_Init(&mc.filter_R, cutoff_freqs_hall, 20);
    
    Filter_Init(&mc.dot_filter_L, cutoff_freqs_hall_dot, 20);
	Filter_Init(&mc.dot_filter_R, cutoff_freqs_hall_dot, 20);

    Filter_Init(&mc.filter_scale, cutoff_freqs_torque_can, 20);

    LOG_INF("Clear Filter Init");
}

void control_init(void){

	/*** Motor Position Limit ***/
    mc.initial_ticks_adm = actual_position;
	
    // q_L is radian
	Position_limit_torque_control(mc.q_L, 1);
	mc.max_limit = limit_wind_ticks_An_left; 	  // ticks
	mc.min_limit = limit_unwind_ticks_An_left;   // ticks

	mc.nominal_q = mc.q_L;
	mc.q_mirror = mc.q_L;
}

// Trajectory Generator
// int find_interval(float x) {
//     int interval = (int)(x / 2);
//     if (interval < 0) {
//         interval = 0;
//     } else if (interval >= 50) {
//         interval = 49;
//     }
//     return interval;
// }

// float generate_trajectory(float x) {
//     int interval = find_interval(x);
//     float a = coefficients[interval][0];
//     float b = coefficients[interval][1];
//     float c = coefficients[interval][2];
//     float d = coefficients[interval][3];
//     float x_local = x - break_points[interval];  // x= 11, break(interval) = 10 -> x_local = 1
//     return a * powf(x_local, 3.0f) + b * powf(x_local, 2.0f) + c * x_local + d;
// }

static inline float compute_offset_sec(float period_sec) {
    return offset_regression_a * period_sec + offset_regression_b;
}

static inline uint32_t clamp_period(uint32_t period_ms) {
    if (period_ms < MIN_PERIOD_MS) return MIN_PERIOD_MS;
    if (period_ms > MAX_PERIOD_MS) return MAX_PERIOD_MS;
    return period_ms;
}

static inline uint32_t median_u32_k(const uint32_t *src, uint8_t n) {
    if (n == 0) return 0;          // 안전한 예외 처리
    if (n > PERIOD_MED_K) n = PERIOD_MED_K;  // 최대값 제한

    uint32_t tmp[PERIOD_MED_K];

    for (uint8_t i = 0; i < n; i++)
        tmp[i] = src[i];

    for (uint8_t i = 1; i < n; i++) {
        uint32_t key = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > key) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }

    return tmp[n / 2];
}

static inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi){
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Shared by the UART test and the main control loop. */
static void collect_log_samples(void)
{
    bool do_sampling = ((mc.time % 1) == 0); // 1 kHz

    /* ===== 1 kHz Sampling ===== */
    if (do_sampling) {
        if (exo_sample_count < LOG_BATCH_EXO) {
            log_qL[exo_sample_count]      = (int16_t)(mc.q_L * 100.0f);
            log_qR[exo_sample_count]      = (int16_t)(mc.q_R * 100.0f);
            log_qMirror[exo_sample_count] = (int16_t)(mc.q_mirror * 100.0f);
            log_torque[exo_sample_count]  = (int16_t)(mc.torque_axis * 10000.0f);

            log_torque_can_L[exo_sample_count] =
                (int16_t)(mc.torque_can_left * 10000.0f);
            log_torque_can_R[exo_sample_count] =
                (int16_t)(mc.torque_can_right * 10000.0f);
            exo_sample_count++;
        }

        if ((emg_sample_count + 1) < LOG_BATCH_EMG) {
            log_tb_emg[emg_sample_count] = mc.L_TB_1;
            log_gt_emg[emg_sample_count] = mc.L_GT_1;
            log_r_tb_emg[emg_sample_count] = mc.R_TB_1;
            log_r_gt_emg[emg_sample_count] = mc.R_GT_1;
            emg_sample_count++;

            log_tb_emg[emg_sample_count] = mc.L_TB_2;
            log_gt_emg[emg_sample_count] = mc.L_GT_2;
            log_r_tb_emg[emg_sample_count] = mc.R_TB_2;
            log_r_gt_emg[emg_sample_count] = mc.R_GT_2;
            emg_sample_count++;
        }
    }
}

static void send_log_frame_if_ready(uint32_t *drop_count)
{
    /* ===== 10ms 패킹 ===== */
    if ((mc.time % 10) == 0) {

        if (exo_sample_count != LOG_BATCH_EXO || emg_sample_count != LOG_BATCH_EMG) {
            return;
        }

        uint8_t frame[FRAME_LEN];
        size_t off = 0;

        frame[off++] = 0xAA;
        frame[off++] = 0x55;

        uint16_t payload_len = 0;

        payload_len += LOG_BATCH_EXO * 4 * sizeof(int16_t); // qL, qR, qMirror, torque_axis
        payload_len += LOG_BATCH_EXO * 2 * sizeof(int16_t); // torque CAN L/R
        payload_len += LOG_BATCH_EMG * LOG_EMG_CHANNEL * EMG_WIRE_SAMPLE_BYTES; // L/R TB/GT EMG

        frame[off++] = payload_len & 0xFF;
        frame[off++] = (payload_len >> 8) & 0xFF;

        memcpy(&frame[off], log_qL, LOG_BATCH_EXO * sizeof(int16_t)); off += LOG_BATCH_EXO * sizeof(int16_t);
        memcpy(&frame[off], log_qR, LOG_BATCH_EXO * sizeof(int16_t)); off += LOG_BATCH_EXO * sizeof(int16_t);
        memcpy(&frame[off], log_qMirror, LOG_BATCH_EXO * sizeof(int16_t)); off += LOG_BATCH_EXO * sizeof(int16_t);
        memcpy(&frame[off], log_torque, LOG_BATCH_EXO * sizeof(int16_t)); off += LOG_BATCH_EXO * sizeof(int16_t);

        memcpy(&frame[off], log_torque_can_L, LOG_BATCH_EXO * sizeof(int16_t)); off += LOG_BATCH_EXO * sizeof(int16_t);
        memcpy(&frame[off], log_torque_can_R, LOG_BATCH_EXO * sizeof(int16_t)); off += LOG_BATCH_EXO * sizeof(int16_t);

        put_emg_i24_be(&frame[off], log_tb_emg);
        off += LOG_BATCH_EMG * EMG_WIRE_SAMPLE_BYTES;
        put_emg_i24_be(&frame[off], log_gt_emg);
        off += LOG_BATCH_EMG * EMG_WIRE_SAMPLE_BYTES;
        put_emg_i24_be(&frame[off], log_r_tb_emg);
        off += LOG_BATCH_EMG * EMG_WIRE_SAMPLE_BYTES;
        put_emg_i24_be(&frame[off], log_r_gt_emg);
        off += LOG_BATCH_EMG * EMG_WIRE_SAMPLE_BYTES;

        int rc = k_msgq_put(&uart_tx_q, frame, K_NO_WAIT);
        if (rc == 0) {
            uart_tx_kick();
        } else {
            // UART TX queue full
            if (drop_count != NULL) {
                (*drop_count)++;
                if ((*drop_count % 100) == 1) {
                    LOG_WRN("UART TX queue full, dropped=%d", *drop_count);
                }
            }
        }

        exo_sample_count = 0;
        emg_sample_count = 0;
    }
}

void UART_Test_step(void){

    static int16_t test_count = 0;
    static uint32_t uart_frame_drop_count = 0;

    mc.q_L = esb_qL;
    mc.q_L = Filter_Low_pass(&mc.filter_L, mc.q_L);

    mc.q_R = esb_qR;
    mc.q_R = Filter_Low_pass(&mc.filter_R, mc.q_R);

    mc.q_mirror  = (float)(test_count + 2000) / 100.0f;

    mc.torque_axis      = 0;
    mc.torque_can_left = esb_torque_L;
    mc.torque_can_right = esb_torque_R;

    // EMG
    mc.L_TB_1 =  esb_tb_emg_1;
    mc.L_GT_1 =  esb_gt_emg_1;
    mc.L_TB_2 =  esb_tb_emg_2;
    mc.L_GT_2 =  esb_gt_emg_2;
    mc.R_TB_1 = esb_r_tb_emg_1;
    mc.R_GT_1 = esb_r_gt_emg_1;
    mc.R_TB_2 = esb_r_tb_emg_2;
    mc.R_GT_2 = esb_r_gt_emg_2;

    mc.time += (int)(SAMPLING_INTERVAL * 1000);
    collect_log_samples();
    send_log_frame_if_ready(&uart_frame_drop_count);
}

static void update_control_sensors(void)
{
    
    // 몇초동안 필터링 안정화한 후에 Position Limitation 수행
    mc.q_L = esb_qL;
    mc.q_L = Filter_Low_pass(&mc.filter_L, mc.q_L);

    mc.q_R = esb_qR;
    mc.q_R = Filter_Low_pass(&mc.filter_R, mc.q_R);

    // if (mc.q_L > deg2rad(130))
    //     mc.q_L = deg2rad(130);
    // else if (mc.q_L < deg2rad(50))
    //     mc.q_L = deg2rad(50);

    // if (mc.q_R > deg2rad(130))
    //     mc.q_R = deg2rad(130);
    // else if (mc.q_R < deg2rad(50))
    //     mc.q_R = deg2rad(50);

    mc.torque_can_left = esb_torque_L;
    mc.torque_can_left = Filter_Low_pass(&mc.filter_L_can, mc.torque_can_left);

    mc.torque_can_right = esb_torque_R;
    mc.torque_can_right = Filter_Low_pass(&mc.filter_R_can, mc.torque_can_right);

    mc.torque_axis = torque_f;
    mc.torque_axis = Measured_Torque_axis(mc.torque_axis);
    mc.torque_axis = Filter_Low_pass(&mc.torque_axis_filter, mc.torque_axis);

    // EMG
    mc.L_TB_1 =  esb_tb_emg_1;
    mc.L_GT_1 =  esb_gt_emg_1;
    
    mc.L_TB_2 =  esb_tb_emg_2;
    mc.L_GT_2 =  esb_gt_emg_2;
    mc.R_TB_1 = esb_r_tb_emg_1;
    mc.R_GT_1 = esb_r_gt_emg_1;
    mc.R_TB_2 = esb_r_tb_emg_2;
    mc.R_GT_2 = esb_r_gt_emg_2;
    
}

static void prepare_compliance_state(void)
{
    // // Compliance
    float torque_scale = esb_torque_L * 1.0;
    Nominal_I = 2 * (1e-6f);
    Nominal_C = 5 * (1e-4f);
    Nominal_K = 10 * (1e-3f);

    // High gain - Lidocaine
    // float torque_scale = esb_torque_L * 2.0;
    // Nominal_I = 1 * (1e-5f);
    // Nominal_C = 1 * (1e-3f);
    // Nominal_K = 30 * (1e-3f);
    
    torque_scale = Filter_Low_pass(&mc.filter_scale, torque_scale);

    // Cantilever torque command
	// mc.torque_ext_clamp = (fabsf(torque_scale) < torque_clamp) ? 0 : torque_scale;
    mc.torque_ext_clamp = 0;

	/* 속도 계산 */
    mc.q_dot_L = (mc.q_L - mc.q_prev_L) / SAMPLING_INTERVAL;
    mc.q_dot_R = (mc.q_R - mc.q_prev_R) / SAMPLING_INTERVAL;

    Filter_Low_pass(&mc.dot_filter_L, mc.q_dot_L);
    Filter_Low_pass(&mc.dot_filter_R, mc.q_dot_R);

}

static void update_control_initialization(void)
{
    // Filter 안정화를 위한 조치
    /* ===== 초기 500ms 안정화 이후 control_init 1회만 실행 ===== */
    if (!control_initialized && mc.time >= INIT_DELAY_MS) {
        control_init();     // 초기 offset, range, safety limit 세팅
        control_initialized = true;
        // 초기 reference or state reset
        mc.time_start = mc.time;
    }

}

static float compute_control_velocity(void)
{
#if defined(CONFIG_CONTRALATERAL_MODE)
    // // Motor Control - Trajectory Mode
    // float df = f_target - f_cur;
    // float df_limit = DF_MAX_PER_SEC * dt;
    // if (df >  df_limit) df =  df_limit;
    // if (df < -df_limit) df = -df_limit;
    // f_cur += df;
    // float x_increment = (100.0 * f_cur) * SAMPLING_INTERVAL;
    // x += x_increment;
    // if (x > 100.0) x -= 100.0;     // [103 - 100] => 3 + x_increment
    // // float Gait_output = generate_trajectory(x); // deg
    // float Gait_output = generate_trajectory(x, RAT_TAG_12);

    // mc.q_R = deg2rad(Gait_output);
    // mc.q_dot_R = (mc.q_R - mc.q_prev_R) / SAMPLING_INTERVAL;

	// Sign State after Hysteresis
	bool vel_pos_now_r = (mc.q_dot_R > +VEL_HYST_RAD_S);
	bool vel_neg_now_r = (mc.q_dot_R < -VEL_HYST_RAD_S);
	uint32_t now_ms_r = mc.time;

    // ---- 피크 후보 처리 ----
	if (now_ms_r >= mc.refractory_util_r) {
		// 속도: + → - 교차 (피크 후보)
		if (mc.was_vel_pos_r && vel_neg_now_r) {
			uint32_t curr_peak_ms = now_ms_r;
			float peak_q = mc.q_R;
			mc.peak_flag = 0;

			// ① 이전 피크보다 충분히 낮은 피크는 작은 봉우리로 간주 → 무시
			bool pass_diff = true;

			if (mc.last_peak_r_ms != 0) {
				float diff = peak_q - mc.last_peak_q;
				if (diff < 0.0f && (-diff) > PEAK_DIFF_THRESH) {
					pass_diff = false;
				}
			}

			// ② 유효 피크일 경우에만 주기 및 이벤트 처리
			if (pass_diff) {
				mc.peak_flag      = 140;
				mc.event_trigger ^= 1u;     // 피크마다 토글

				if (mc.last_peak_r_ms != 0) {
					mc.period_ms = curr_peak_ms - mc.last_peak_r_ms;
	    			
					// 버퍼 저장
					mc.period_buf[mc.period_idx] = mc.period_ms;
					mc.period_idx++;
					if (mc.period_idx >= PERIOD_MED_K) mc.period_idx = 0;
					if (mc.period_count < PERIOD_MED_K) mc.period_count++;

					// 중앙값 주기
					mc.med_ms = median_u32_k(mc.period_buf, mc.period_count);

					// 오프셋 계산
					mc.offset_sec = compute_offset_sec((float)mc.med_ms / 1000.0f);
					mc.offset_sec = mc.offset_sec * 1000.0f;
					mc.target_offset_ms = clamp_u16(
						(uint16_t)(mc.offset_sec < 0 ? 0 : mc.offset_sec),
						MIN_OFFSET_MS, MAX_OFFSET_MS
					);
					mc.last_period_r_ms = mc.period_ms;
				}

				// 상태 갱신
				mc.last_peak_q = peak_q;
				mc.last_peak_r_ms = curr_peak_ms;
				mc.refractory_util_r = now_ms_r + REFRACT_MS; // 100 ms 유지
			}
		}
		else {
			mc.peak_flag = 0;
		}
	}

	// Velocity Extreme Detection
	if (vel_pos_now_r) mc.was_vel_pos_r = true;
	else if (vel_neg_now_r) mc.was_vel_pos_r = false;

	// Current offset -> Target offset
	if (mc.offset_ms + OFFSET_SLEW_MS < mc.target_offset_ms) {
		mc.offset_ms += OFFSET_SLEW_MS;
	} else if (mc.offset_ms > mc.target_offset_ms + OFFSET_SLEW_MS) {
		mc.offset_ms -= OFFSET_SLEW_MS;
	} else {
		mc.offset_ms = mc.target_offset_ms;
	}

	// Mirror Buffer Store
	int16_t scaled = (int16_t)lrintf(mc.q_R * buffer_scale);
	mc.angle_buffer[mc.head] = scaled;
	mc.head = (mc.head + 1) % ANG_BUF_CAP;
	if (mc.filled < ANG_BUF_CAP) mc.filled++;

	// Index extrude after calculate delay sample number
	uint32_t sample_ms = (uint32_t)lrintf(SAMPLING_INTERVAL * 1000.0f); // 보통 1ms
	uint32_t delay_req      = mc.offset_ms / sample_ms;

	// 2) 가용 상한 계산
	uint32_t cap_limit      = (ANG_BUF_CAP > 0) ? (ANG_BUF_CAP - 1) : 0;
	uint32_t filled_limit   = (mc.filled > 0) ? (mc.filled - 1) : 0;

	// 3) 실제 적용 지연 = 요청치 포화
	uint32_t delay_applied  = delay_req;
	if (delay_applied > cap_limit)    delay_applied = cap_limit;
	if (delay_applied > filled_limit) delay_applied = filled_limit;

	// 4) 항상 q_mirror를 산출 (가용한 만큼만 뒤로 감기)
	uint16_t read_idx = (uint16_t)((mc.head + ANG_BUF_CAP - 1 - (uint16_t)delay_applied) % ANG_BUF_CAP);
	int16_t raw = mc.angle_buffer[read_idx];

	mc.q_mirror  = (float)raw / (float)buffer_scale;
	mc.torque_admittance = K_adm *((mc.nominal_q_dot-mc.q_dot_L) + Kp_adm * (mc.nominal_q-mc.q_L));
	mc.torque_admittance = 0.001f*mc.torque_admittance;

	// Nominal Plant - Desired (Radian)
	mc.nominal_q_ddot = (mc.torque_ext_clamp - mc.torque_admittance - Nominal_C * mc.nominal_q_dot - Nominal_K * (mc.nominal_q - mc.q_mirror) ) / (Nominal_I);
    mc.nominal_q_dot = mc.nominal_q_dot + SAMPLING_INTERVAL * mc.nominal_q_ddot;
	mc.nominal_q = mc.nominal_q + SAMPLING_INTERVAL * mc.nominal_q_dot;

	// Clamping Nominal Pose
	if (mc.nominal_q > theta_max_axial){
		mc.nominal_q = theta_max_axial;
	}else if(mc.nominal_q < theta_min_axial){
		mc.nominal_q = theta_min_axial;
	}

	// Command_vel Clamping
	float command_vel_float = Kp * (mc.q_L - mc.nominal_q) + Kv * (mc.q_dot_L - mc.nominal_q_dot);

#elif defined(CONFIG_FIXED_MODE)
    // Motor Control - Trajectory Mode
    float df = f_target - f_cur;
    float df_limit = DF_MAX_PER_SEC * dt;
    if (df >  df_limit) df =  df_limit;
    if (df < -df_limit) df = -df_limit;
    f_cur += df;
    float x_increment = (100.0 * f_cur) * SAMPLING_INTERVAL;
    x += x_increment;
    if (x > 100.0) x -= 100.0;     // [103 - 100] => 3 + x_increment
    float Gait_output = generate_trajectory(x, RAT_TAG_12); // deg
    float An_R = Gait_output;

    mc.q_fixed = deg2rad(An_R);
    mc.q_dot_fixed = (mc.q_fixed - mc.q_prev_q_fixed) / SAMPLING_INTERVAL;
    mc.q_prev_q_fixed = mc.q_fixed;

    // mc.q_L = mc.q_fixed;
    // Command_vel Clamping
	float command_vel_float = Kp_fixed * (mc.q_L - mc.q_fixed) + Kd_fixed * (mc.q_dot_L - mc.q_dot_fixed);
#endif
    return command_vel_float;
}

static float limit_control_velocity(float command_vel_float)
{
	// Velocity Damping Saturation
	float q_remain_UP = mc.max_limit - mc.ticks_diff;
	float q_dot_max;

	// Velocity Upper Bound
	if(q_remain_UP <= qi){
        // Ratio 분모 보호용 (qi == qs 시, 폭발 방지)
        float denom = (qi - qs);
        if (fabsf(denom) < 1e-6f) denom = 1e-6f;

		float ratio = (q_remain_UP - qs) / denom;
		ratio = fmax(ratio, 0.0);
		q_dot_max = xi * ratio;
	} else{
		q_dot_max = MAXIMUM_VEL_RPM;
	}

	float q_remain_LW = mc.ticks_diff - mc.min_limit;
	float q_dot_min;

	if(q_remain_LW <= qi){
        // Ratio 분모 보호용 (qi == qs 시, 폭발 방지)
        float denom = (qi - qs);
        if (fabsf(denom) < 1e-6f) denom = 1e-6f;
        
		float ratio = (q_remain_LW - qs) / denom;
		ratio = fmax(ratio, 0.0f);
		q_dot_min = -xi * ratio;
	}else{
		q_dot_min = -MAXIMUM_VEL_RPM;
	}

    // Float 타입을 int16_t로 변경했을 때, wrap-around 문제가 발생할 수 있다.
    // ex) Command_Vel = 40,000 --> (int16_t) 40,000 = -25536 
    command_vel_float = fmin(fmax(command_vel_float, q_dot_min), q_dot_max);

    // Filter (float 상태)
    command_vel_float = Filter_Low_pass(&mc.motor_command_filter, command_vel_float);
    
    // NaN / Inf 체크  
    if (isnan(command_vel_float) || isinf(command_vel_float)) {
        command_vel_float = 0;
    }
    return command_vel_float;
}

static void send_motor_command(float command_vel_float)
{
    // Int 변환
    int32_t cmd_vel_init =  (int32_t)command_vel_float;

    // 지역변수 can 구조체
    struct can_frame tx;

    if (control_initialized == true) {
        pack_pdo_CSV(&tx, COB_ID_PDO3_RX + 1, cmd_vel_init);  // velocity = 200 rpm
	    send_frame(&tx);   // 너가 이미 만든 can_send() 래퍼
        // pack_pdo_CSV(&tx, COB_ID_PDO3_RX + 1, 0);  // velocity = 200 rpm
	    // send_frame(&tx);   // 너가 이미 만든 can_send() 래퍼
    }

}

static void advance_control_state(void)
{
    
	/* Exo-skeleton Motor Position */
	mc.ticks_diff = actual_position - mc.initial_ticks_adm;
	mc.prev_ticks_diff = mc.ticks_diff;

    mc.q_prev_L = mc.q_L;
	mc.q_prev_R = mc.q_R;

    mc.time += (int)(SAMPLING_INTERVAL * 1000);
}

void Mirror_complinace_control_step(const struct device *can_dev)
{
    /* CAN transmission uses the existing send_frame() device binding. */
    (void)can_dev;

    update_control_sensors();
    prepare_compliance_state();
    update_control_initialization();

    float command_velocity = compute_control_velocity();
    command_velocity = limit_control_velocity(command_velocity);
    send_motor_command(command_velocity);

    advance_control_state();
    collect_log_samples();
    send_log_frame_if_ready(NULL);
}

// Position Limit
float L_Norm(float An_deg, uint8_t exo_type){
    float theta_rad = deg2rad(90) - deg2rad(An_deg);
    float Rp[2], L[2];

    Rp[0] = cosf(theta_rad) * p[0] - sinf(theta_rad) * p[1];
    Rp[1] = sinf(theta_rad) * p[0] + cosf(theta_rad) * p[1];

    L[0] = b[0] - Rp[0];
    L[1] = b[1] - Rp[1];
    float L_norm = sqrtf(L[0]*L[0] + L[1]*L[1]);
    return L_norm;
}

void Position_limit_torque_control(float An_rad, uint8_t exo_type){
	float margin_ticks_wind, margin_ticks_unwind;

	/* Margin */
	margin_ticks_unwind = 300;
	margin_ticks_wind = 300;

	float limit_unwind_angle_An, limit_wind_angle_An;
	float L_upper_norm = L_Norm(rad2deg(theta_max_axial), exo_type);
	float L_lower_norm = L_Norm(rad2deg(theta_min_axial), exo_type);

    // q_L = An is radian
	float L_An_norm = L_Norm(rad2deg(An_rad), exo_type);

    limit_unwind_angle_An = (L_An_norm - L_upper_norm)/Pulley_Radius;
	limit_wind_angle_An = (L_An_norm - L_lower_norm)/Pulley_Radius;

    limit_unwind_ticks_An_left = rad2deg(limit_unwind_angle_An) * TICKS_PER_DEGREE - margin_ticks_unwind;
	limit_wind_ticks_An_left = rad2deg(limit_wind_angle_An) * TICKS_PER_DEGREE + margin_ticks_wind;

    LOG_INF("\n Motor Position Boundary %d, %d, %d, %d", (int16_t)limit_unwind_ticks_An_left, (int16_t)limit_wind_ticks_An_left, (int16_t)limit_unwind_ticks_An_right, (int16_t)limit_wind_ticks_An_right);
}






