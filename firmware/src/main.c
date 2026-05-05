#include "parameter.h"
#include "adc_pwm.h"
#include "ble_service.h"

#ifndef ADC_THREAD_STACK
#define ADC_THREAD_STACK 2048
#endif
#ifndef BLE_THREAD_STACK
#define BLE_THREAD_STACK 1024
#endif
#ifndef ADC_THREAD_PRIO
#define ADC_THREAD_PRIO 4
#endif
#ifndef BLE_THREAD_PRIO
#define BLE_THREAD_PRIO 3
#endif

K_THREAD_STACK_DEFINE(adc_stack, ADC_THREAD_STACK);
K_THREAD_STACK_DEFINE(ble_stack, BLE_THREAD_STACK);
static struct k_thread adc_th;
static struct k_thread ble_th;

static void adc_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    adc_pwm_init();
}

static void ble_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    (void)ble_init_and_advertise();
    while (1) {
        k_sleep(K_SECONDS(1));
    }
}

void main(void)
{
    (void)bt_set_name(APP_DEVICE_NAME);

    k_thread_create(&adc_th, adc_stack, K_THREAD_STACK_SIZEOF(adc_stack),
                    adc_thread, NULL, NULL, NULL, ADC_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&adc_th, "adc_pwm");

    k_thread_create(&ble_th, ble_stack, K_THREAD_STACK_SIZEOF(ble_stack),
                    ble_thread, NULL, NULL, NULL, BLE_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&ble_th, "ble_core");

    while (1) {
        k_sleep(K_FOREVER);
    }
}