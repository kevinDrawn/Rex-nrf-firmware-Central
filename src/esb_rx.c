#include <esb_rx.h>
LOG_MODULE_REGISTER(esb_rx, LOG_LEVEL_INF);

static int16_t rx_buf_L[RX_BUFFER_SIZE];
static int16_t rx_buf_R[RX_BUFFER_SIZE];
static int rx_count = 0;

static int64_t last_print_time = 0;   // ms

volatile float esb_qL = 0.0f;
volatile float esb_qR = 0.0f;

volatile float esb_torque_L = 0.0f;
volatile float esb_torque_R = 0.0f;

volatile int32_t esb_tb_emg_1 = 0.0f;
volatile int32_t esb_gt_emg_1 = 0.0f;
volatile int32_t esb_tb_emg_2 = 0.0f;
volatile int32_t esb_gt_emg_2 = 0.0f;

static volatile uint32_t esb_rx_count = 0;
static volatile uint32_t esb_rx_underflow_count = 0;
static volatile uint32_t esb_rx_emg_seq_error_count = 0;

static volatile uint32_t esb_rx_loss_count = 0;

static uint16_t last_seq = 0;
static bool first_packet = true;

// Control Step Trigger - ESB 데이터 수신 전까지 제어 시작 안함
volatile bool esb_angle_ready = false;

// RX 버퍼
static struct esb_payload rx_payload;
static inline int16_t bytes_to_i16(uint8_t high, uint8_t low)
{
    return (int16_t)((high << 8) | low);
}

static inline int32_t bytes_to_i32(uint8_t b0,
                                   uint8_t b1,
                                   uint8_t b2,
                                   uint8_t b3)
{
    uint32_t v = ((uint32_t)b0 << 24) |
                 ((uint32_t)b1 << 16) |
                 ((uint32_t)b2 << 8)  |
                 ((uint32_t)b3);

    return (int32_t)v;
}

float deg2rad(float deg){
	return deg * (3.141592 / 180.0f);
}

float rad2deg(float rad){
	return rad * (180.0f / 3.141592f);
}

static uint16_t bytes_to_u16(uint8_t msb, uint8_t lsb)
{
    return ((uint16_t)msb << 8) | (uint16_t)lsb;
}

// ESB Driver에서 발생한 이벤트 받아 처리
static void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id) {
	// ACK TX 성공
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS EVENT");
		break;
	// TX 실패
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED EVENT");
		break;
	// PTX -> PRX 패킷 도착
case ESB_EVENT_RX_RECEIVED:
    while (esb_read_rx_payload(&rx_payload) == 0) {

        if (rx_payload.length < DATA_LENGTH) {
            LOG_ERR("Payload too short! len=%d", rx_payload.length);
            continue;
        }

		esb_rx_count++;

		uint16_t seq = bytes_to_u16(rx_payload.data[0], rx_payload.data[1]);
		uint8_t flags = rx_payload.data[2];	

		int16_t L_Raw = bytes_to_i16(rx_payload.data[4], rx_payload.data[5]);
		int16_t R_Raw = bytes_to_i16(rx_payload.data[6], rx_payload.data[7]);

		int16_t L_torque = bytes_to_i16(rx_payload.data[8], rx_payload.data[9]);
		int16_t R_torque = bytes_to_i16(rx_payload.data[10], rx_payload.data[11]);

		int32_t TB_EMG_1 =
			bytes_to_i32(rx_payload.data[12],
						rx_payload.data[13],
						rx_payload.data[14],
						rx_payload.data[15]);

		int32_t GT_EMG_1 =
			bytes_to_i32(rx_payload.data[16],
						rx_payload.data[17],
						rx_payload.data[18],
						rx_payload.data[19]);

		int32_t TB_EMG_2 =
			bytes_to_i32(rx_payload.data[20],
						rx_payload.data[21],
						rx_payload.data[22],
						rx_payload.data[23]);

		int32_t GT_EMG_2 =
			bytes_to_i32(rx_payload.data[24],
						rx_payload.data[25],
						rx_payload.data[26],
						rx_payload.data[27]);
						
        if (flags & PAYLOAD_FLAG_EMG_UNDERFLOW) {
            esb_rx_underflow_count++;
        }

        if (flags & PAYLOAD_FLAG_EMG_SEQUENCE_ERROR) {
            esb_rx_emg_seq_error_count++;
        }

		if (first_packet) {

			first_packet = false;

		} else {

			uint16_t expected_seq = (uint16_t)(last_seq + 1);

			if (seq != expected_seq) {

				uint16_t lost =
					(uint16_t)(seq - expected_seq);

				esb_rx_loss_count += lost;
			}
		}

		last_seq = seq;

#if defined(CONFIG_HALL_MODE)

		esb_qL = deg2rad((float) L_Raw / 100.0f);
        esb_qR = deg2rad((float) R_Raw / 100.0f);

		// Print raw data
		// int16_t norm_qL = (float) L_Raw / 100.0f;
        // int16_t norm_qR = (float) R_Raw / 100.0f;
		// LOG_INF("%d %d", L_Raw, R_Raw);
		// LOG_INF("%d %d", norm_qL, norm_qR);
#endif

#if defined(CONFIG_CAN_MODE)
		// Scaling + 공유 변수 업데이트 - float atomic 읽기 race tracing 문제 해결
        esb_torque_L = Measured_Torque_Character_cantilever(L_torque, CHAR_M);
        esb_torque_R = Measured_Torque_Character_cantilever(R_torque, CHAR_O);

		// Calibration 용 ADC 값 출력
		// esb_torque_L = L_torque;
        // esb_torque_R = R_torque;

		// LOG_INF("%d %d", L_torque, R_torque);
        // esb_torque_L = (float) L_torque;
        // esb_torque_R = (float) R_torque;
#endif

#if defined(CONFIG_EMG_MODE)
        esb_tb_emg_1 = TB_EMG_1;
        esb_gt_emg_1 = GT_EMG_1;
        esb_tb_emg_2 = TB_EMG_2;
        esb_gt_emg_2 = GT_EMG_2;

		// LOG_INF("TB_EMG_1: %d, GT_EMG_1: %d", esb_tb_emg_1, esb_gt_emg_1);
#endif
		// Control Loop 수행 시작 준비 완료
		esb_angle_ready = true;   // 최신 각도 도착 표시

        // /* 버퍼 저장 */
        // if (rx_count < RX_BUFFER_SIZE) {
        //     rx_buf_L[rx_count] = L_Raw;
        //     rx_buf_R[rx_count] = R_Raw;
        //     rx_count++;
        // }

        // /* 1초 지났는지 체크하여 1000개 dump */
        // int64_t now = k_uptime_get();

        // if (now - last_print_time >= 1000) {
        //     last_print_time = now;

        //     /* 실제 수신 속도 계산 */
        //     int received = rx_count;
        //     float rx_rate_hz = received / 1.0f;   // 1초 기준

        //     LOG_INF("=== 1초 동안 수신된 패킷: %d개 ===",
        //             received);

        //     /* 원하는 형태로 dump (처음 5개만 예시로 표시) */
        //     for (int i = 0; i < MIN(received, 5); i++) {
        //         LOG_INF("[%d] L=%d, R=%d", i, rx_buf_L[i], rx_buf_R[i]);
        //     }

        //     /* 버퍼 리셋 */
        //     rx_count = 0;
        // }
    }
    break;
	}
}

// 고주파 (HF) 클록 시작 함수
#if defined(CONFIG_CLOCK_CONTROL_NRF)
int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err);

	LOG_INF("HF clock started");
	return 0;
}
#else
BUILD_ASSERT(false, "No Clock Control driver");
#endif /* defined(CONFIG_CLOCK_CONTROL_NRF2) */

// ESB 초기화 함수
int esb_initialize(void)
{
	int err;
	/* These are arbitrary default addresses. In end user products
	 * different addresses should be used for each set of devices.
	 */
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t prefix[1] = {0xE7};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	// DPL - Dynamic Payload Length - PTX PRX 보내는 패킷 다를 때 사용
	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.bitrate = ESB_BITRATE_2MBPS;
	config.mode = ESB_MODE_PRX;
	config.event_handler = event_handler;
	// True로 설정하면, 수신 즉시 자동 ACK 전송한다.
	config.selective_auto_ack = true;
	if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING)) {
		config.use_fast_ramp_up = true;
	}

	err = esb_init(&config);
	if (err) {
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		return err;
	}

	err = esb_set_prefixes(prefix, 1);
	if (err) {
		return err;
	}

	LOG_INF("Clear ESB Initialize"); 
	return 0;
}

uint32_t get_esb_rx_count(void)
{
    return esb_rx_count;
}

uint32_t get_esb_rx_underflow_count(void)
{
    return esb_rx_underflow_count;
}

uint32_t get_esb_rx_emg_seq_error_count(void)
{
    return esb_rx_emg_seq_error_count;
}

uint32_t get_esb_rx_loss_count(void)
{
    return esb_rx_loss_count;
}