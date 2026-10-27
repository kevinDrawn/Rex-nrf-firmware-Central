#pragma once

// SSADC + PPI
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <helpers/nrfx_gppi.h>

#if defined(DPPI_PRESENT)
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif
#include <zephyr/kernel.h>
#include <torque_cyl.h>
#include <motor_control.h>
#include <esb_rx.h>

// ADC + SAADC
#define ADC_channel_num 1
#define SAADC_SAMPLE_INTERVAL_US 1000							 // 1000 us - 1 ms / 1 kHz
#define SAADC_BUFFER_SIZE 1	// 1 ms - 1 Sample - 정밀한 1 ms timer 구성 완료
#define SAADC_INPUT_PIN NRF_SAADC_INPUT_AIN0 // p0.02 nRF 52DK
// #define CENTRAL_RING_SIZE 4

// Timer
#define TIMER_INSTANCE_NUMBER 1

// Declare the struct to hold the configuration for the SAADC Channel used to sample the data
extern nrfx_saadc_channel_t channel;	 // 0 채널

// Declare an instance of nrfx_timer for TIMER2
extern const nrfx_timer_t timer_instance;
extern volatile float torque_f;

void configure_timer(void);
void saadc_event_handler(nrfx_saadc_evt_t const *p_event);
void configure_saadc(void);
void configure_ppi(void);
