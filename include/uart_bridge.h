#pragma once
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <motor_control.h>

/* ==========================================================
 * SAADC 버퍼 크기 설정
 * ----------------------------------------------------------
 * - Hall 모드만 또는 CAN 모드만 활성화 시 2채널 → 20샘플
 * - 두 모드 모두 활성화 시 4채널 → 40샘플
 * ========================================================== */
#if defined(CONFIG_HALL_MODE) && defined(CONFIG_CAN_MODE)
    #define SAADC_BUFFER_SIZE_UART 40
#elif defined(CONFIG_CONTROL_MODE)
    #define SAADC_BUFFER_SIZE_UART 30
#else
    #define SAADC_BUFFER_SIZE_UART 30
#endif
#define CENTRAL_SAADC_BUFFER_SIZE 10

#define FRAME_HEADER_LEN 4

#define PAYLOAD_LEN \
    (LOG_BATCH_EXO * 6 * sizeof(int16_t) + \
     LOG_BATCH_EMG * 2 * sizeof(int32_t))

#define FRAME_LEN (FRAME_HEADER_LEN + PAYLOAD_LEN)

#define RX_IDLE_TIMEOUT_MS 30
extern const struct device *uart_dev;

/* PC로 보낼 프레임(보드→PC) */
extern struct k_msgq uart_tx_q;

/* 초기화 */
int uart_bridge_init(void);

/* 큐에 넣은 뒤 다음 TX를 깨우기 */
void uart_tx_kick(void);
