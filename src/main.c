/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <esb_rx.h>
#include <adc_sampler.h>
#include <can.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
	int err;
	LOG_INF("Start Project: PSH Rodent Exoskeleton");

	// HFCLK ON 
	err = clocks_start();
	if (err) {
		return 0;
	}
	k_busy_wait(200);

	// ESB Setting
	err = esb_initialize();
	if (err) {
		LOG_ERR("ESB initialization failed, err %d", err);
		return 0;
	}

	// RX 시작
	err = esb_start_rx();
	if (err) {
		LOG_ERR("RX setup failed, err %d", err);
		return 0;
	}

	/* UART Init */
	int rc = uart_bridge_init();
    if (rc) {
        LOG_ERR("uart_bridge_init rc=%d\n", rc);
        return -1;  /* 실패 시 이후 경로 스킵 */
    }

	/* CAN Init */
	can_init();

	/* Controller Filter Init */
	control_filter_init();

	// SAADC Setup
	configure_timer();
	configure_saadc();
	configure_ppi();

	uint32_t prev_can_send_fail_count = 0;
	while(true){
		k_sleep(K_SECONDS(1));
		uint32_t can_send_fail_count = get_can_send_fail_count();
		LOG_INF("CAN send failed: last_1s=%u total=%u",
			can_send_fail_count - prev_can_send_fail_count,
			can_send_fail_count);
		prev_can_send_fail_count = can_send_fail_count;
		
		// LOG_INF("RX=%u LOSS=%u UNDER=%u EMG_SEQ_ERR=%u",
		// 		get_esb_rx_count(),
		// 		get_esb_rx_loss_count(),
		// 		get_esb_rx_underflow_count(),
		// 		get_esb_rx_emg_seq_error_count());
	}
	/* return to idle thread */
	return 0;
}
