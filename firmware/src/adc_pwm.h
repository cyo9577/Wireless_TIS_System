#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void adc_pwm_set_start(bool on);
void adc_pwm_set_targets(uint16_t th0, uint16_t th1);
void adc_pwm_set_mux(uint8_t mux);

uint16_t adc_pwm_get_last_p2p_ch0(void);
uint16_t adc_pwm_get_last_p2p_ch1(void);

uint8_t  adc_pwm_get_amp0_code(void);
uint8_t  adc_pwm_get_amp1_code(void);

void adc_pwm_set_freqs(uint16_t f0_hz, uint16_t f1_hz);

#ifdef __cplusplus
}
#endif
