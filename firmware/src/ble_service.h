#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

int  ble_init_and_advertise(void);
bool ble_notify_enabled(void);
int  ble_notify_status(uint8_t p2p0, uint16_t p2p1, uint8_t amp0, uint8_t amp1);
void adc_pwm_set_freqs(uint16_t f0_hz, uint16_t f1_hz);

#ifdef __cplusplus
}
#endif
