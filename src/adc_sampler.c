#include <adc_sampler.h>

LOG_MODULE_REGISTER(adc_sampler, LOG_LEVEL_INF);

// Declare the buffers for the SAADC
int16_t saadc_sample_buffer[2][SAADC_BUFFER_SIZE];				 // DMA 용 Double Buffer 등으로 사용되는 실시간 SAADC 샘플 버퍼
const nrfx_timer_t timer_instance = NRFX_TIMER_INSTANCE(TIMER_INSTANCE_NUMBER);

uint32_t saadc_current_buffer = 0;
static nrfx_saadc_channel_t channel_cyl = NRFX_SAADC_DEFAULT_CHANNEL_SE(SAADC_INPUT_PIN, 0);	 // 0 채널

volatile float torque_f = 0.0f;
// K_WORK_DEFINE(ctrl_work, Mirror_complinace_control_step);
K_WORK_DEFINE(ctrl_work, Mirror_complinace_control_step);

void saadc_event_handler(nrfx_saadc_evt_t const *p_event)
{
	nrfx_err_t err;
	switch (p_event->type)
	{
	case NRFX_SAADC_EVT_READY:
		/* STEP 5.1 - Buffer is ready, timer (and sampling) can be started. */
		nrfx_timer_enable(&timer_instance);
		break;

	case NRFX_SAADC_EVT_BUF_REQ:
		err = nrfx_saadc_buffer_set(saadc_sample_buffer[(saadc_current_buffer++) % 2], SAADC_BUFFER_SIZE);
		if (err != NRFX_SUCCESS)
		{
			LOG_ERR("nrfx_saadc_buffer_set error: %08x", (unsigned int)err);
			return;
		}
		break;

	case NRFX_SAADC_EVT_DONE:
	{
		int16_t *buf = p_event->data.done.p_buffer;
	
		// Torque 정보만 전역 변수에 저장
		torque_f = (float) buf[0];

		// ESB 수신 시, Control Trigger 발동
		if (esb_angle_ready) {
			k_work_submit(&ctrl_work);
    	}
		break;
	}

	default:
		LOG_WRN("Unhandled SAADC evt %d", (int)p_event->type);
		break;
	}
}

void configure_saadc(void)
{
	nrfx_err_t err;
	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
				DT_IRQ(DT_NODELABEL(adc), priority),
				nrfx_isr, nrfx_saadc_irq_handler, 0);

	err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
	if (err != NRFX_SUCCESS)
	{
		LOG_ERR("nrfx_saadc_init error: %08x", err);
		return;
	}

	channel_cyl.channel_config.gain = NRF_SAADC_GAIN1_6;

	nrfx_saadc_channel_t channels[1];
	channels[0] = channel_cyl;

	// 채널 구성
	err = nrfx_saadc_channels_config(channels, 1);
	if (err != NRFX_SUCCESS)
	{
		LOG_ERR("nrfx_saadc_channels_config error: %08x", err);
		return;
	}

	nrfx_saadc_adv_config_t saadc_adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
	err = nrfx_saadc_advanced_mode_set(BIT(0), 
									   NRF_SAADC_RESOLUTION_12BIT,
									   &saadc_adv_config,
									   saadc_event_handler); 
	if (err != NRFX_SUCCESS)
	{
		LOG_ERR("nrfx_saadc_advanced_mode_set error: %08x", err);
		return;
	}

	err = nrfx_saadc_buffer_set(saadc_sample_buffer[0], SAADC_BUFFER_SIZE);
	if (err != NRFX_SUCCESS)
	{
		LOG_ERR("nrfx_saadc_buffer_set error: %08x", err);
		return;
	}

	err = nrfx_saadc_buffer_set(saadc_sample_buffer[1], SAADC_BUFFER_SIZE);
	if (err != NRFX_SUCCESS)
	{
		LOG_ERR("nrfx_saadc_buffer_set error: %08x", err);
		return;
	}

	err = nrfx_saadc_mode_trigger();
	if (err != NRFX_SUCCESS)
	{
		LOG_ERR("nrfx_saadc_mode_trigger error: %08x", err);
		return;
	}

	LOG_INF("Clear Configure SAADC");
}

void configure_ppi(void)
{
	nrfx_err_t err;
    const uint8_t ppi_ch_sample = 10;   // Timer → SAMPLE
    const uint8_t ppi_ch_start  = 11;   // END → START

	nrfx_gppi_channels_disable(BIT(ppi_ch_sample));
	nrfx_gppi_channels_disable(BIT(ppi_ch_start));
	
	nrfx_gppi_channels_disable(BIT(ppi_ch_sample));
	nrfx_gppi_channels_disable(BIT(ppi_ch_start));
	
	nrfx_gppi_channel_endpoints_setup(ppi_ch_sample,
		nrfx_timer_compare_event_address_get(&timer_instance, NRF_TIMER_CC_CHANNEL0),
		nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));

	nrfx_gppi_channel_endpoints_setup(ppi_ch_start,
		nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
		nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START));

	nrfx_gppi_channels_enable(BIT(ppi_ch_sample) | BIT(ppi_ch_start));
	
	LOG_INF("Clear Configure PPI");
}

void configure_timer(void)
{
	nrfx_err_t err;
	nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(1000000); // 1 MHz Timer

	err = nrfx_timer_init(&timer_instance, &timer_config, NULL); // no interrupt - NULL 타이머 이벤트만 만들고 인터럽트 발생 x
	if (err != NRFX_SUCCESS)
	{
		LOG_ERR("nrfx_timer_init error: %08x", err);
		return;
	}

	uint32_t timer_ticks = nrfx_timer_us_to_ticks(&timer_instance, SAADC_SAMPLE_INTERVAL_US);
	nrfx_timer_extended_compare(&timer_instance, NRF_TIMER_CC_CHANNEL0, timer_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
	LOG_INF("Clear Configure Timer");
}