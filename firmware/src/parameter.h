#ifndef PARAMETER_H__
#define PARAMETER_H__

#include <zephyr/kernel.h>
#include <math.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <hal/nrf_gpio.h>

#define Device 1

#if Device == 1
#define APP_DEVICE_NAME "TIS_System"
#endif

#define GPIO_PIN0 5
#define GPIO_PIN1 29
#define GPIO_PWR  32

#define PWM_INST_IDX0 0
#define LED1_PIN      21

#define PWM_INST_IDX1 1
#define LED2_PIN      40

#define MAX_SAMPLES     200
#define PWM_PERIOD      160
#define PI              3.14159265358979323846f
#define PWM_FREQ0       1000.0f
#define NUM_SAMPLES0    100
#define PWM_FREQ1       1010.0f
#define NUM_SAMPLES1    99

#define ANALOG_INPUT_TO_SAADC_AIN(x) ((x) + 1)
#define ANALOG_INPUT_TO_COMP_AIN(x)   (x)
#define ANALOG_INPUT_A0 2
#define CH0_AIN ANALOG_INPUT_TO_SAADC_AIN(ANALOG_INPUT_A0)
#define ANALOG_INPUT_A1 3
#define CH1_AIN ANALOG_INPUT_TO_SAADC_AIN(ANALOG_INPUT_A1)

#define SAADC_CHANNEL_SE_ACQ(_pin_p, _pin_n, _index, _mode) \
{ .channel_config = { \
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED, \
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED, \
    .gain       = NRF_SAADC_GAIN1_4, \
    .reference  = NRF_SAADC_REFERENCE_VDD4, \
    .acq_time   = NRF_SAADC_ACQTIME_3US, \
    .mode       = NRF_SAADC_MODE_SINGLE_ENDED, \
    .burst      = NRF_SAADC_BURST_DISABLED, \
  }, \
  .pin_p = (nrf_saadc_input_t)_pin_p, \
  .pin_n = NRF_SAADC_INPUT_DISABLED, \
  .channel_index = _index, \
}

#define ADC_RESOLUTION       NRF_SAADC_RESOLUTION_12BIT
#define CHUNK_POOL_SIZE 24
#define INTERNAL_TIMER_FREQ      16000000UL
#define SAADC_SAMPLE_FREQUENCY   20000UL
#define INTERNAL_TIMER_CC (INTERNAL_TIMER_FREQ / SAADC_SAMPLE_FREQUENCY)

#define CHUNK_SAMPLES 2000U
#define CHUNK_BYTES   (CHUNK_SAMPLES * sizeof(uint16_t))

#ifndef NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY
#define NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY 7
#endif

#define REQ_CONN_INTERVAL_MIN  6
#define REQ_CONN_INTERVAL_MAX 24
#define REQ_CONN_LATENCY       0
#define REQ_CONN_TIMEOUT     400

#endif
