#pragma once

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <math.h>

#include <torque_cyl.h>

#define RX_BUFFER_SIZE 1000

#define DATA_LENGTH 28
#define PAYLOAD_FLAG_EMG_UNDERFLOW      BIT(0)
#define PAYLOAD_FLAG_EMG_SEQUENCE_ERROR BIT(1)

extern volatile float esb_qL;
extern volatile float esb_qR;

extern volatile float esb_torque_L;
extern volatile float esb_torque_R;

extern volatile int32_t esb_tb_emg_1;
extern volatile int32_t esb_gt_emg_1;
extern volatile int32_t esb_tb_emg_2;
extern volatile int32_t esb_gt_emg_2;

extern volatile bool esb_angle_ready;

int clocks_start(void);
int esb_initialize(void);

float deg2rad(float deg);
float rad2deg(float rad);

uint32_t get_esb_rx_count(void);
uint32_t get_esb_rx_underflow_count(void);
uint32_t get_esb_rx_emg_seq_error_count(void);
uint32_t get_esb_rx_loss_count(void);