#include "parameter.h"
#include "adc_pwm.h"
#include "ble_service.h"

#include <nrfx_pwm.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_saadc.h>
#include <hal/nrf_dppi.h>

#ifndef TARGET_P2P_DEFAULT
#define TARGET_P2P_DEFAULT 0u
#endif

static volatile uint16_t g_amp0 = 0;
static volatile uint16_t g_amp1 = 0;
static volatile uint16_t g_th0  = TARGET_P2P_DEFAULT;
static volatile uint16_t g_th1  = TARGET_P2P_DEFAULT;
static volatile uint8_t  g_mux  = 0;
static volatile bool     g_started = false;

static volatile uint16_t g_last_p2p0 = 0;
static volatile uint16_t g_last_p2p1 = 0;

static volatile uint16_t g_freq0_hz = (uint16_t)(PWM_FREQ0);
static volatile uint16_t g_freq1_hz = (uint16_t)(PWM_FREQ1);

static volatile uint16_t g_nsamp0 = NUM_SAMPLES0;
static volatile uint16_t g_nsamp1 = NUM_SAMPLES1;

static nrfx_pwm_t s_pwm0 = NRFX_PWM_INSTANCE(PWM_INST_IDX0);
static nrfx_pwm_t s_pwm1 = NRFX_PWM_INSTANCE(PWM_INST_IDX1);

static nrf_pwm_values_individual_t s_vals0[MAX_SAMPLES];
static nrf_pwm_values_individual_t s_vals1[MAX_SAMPLES];

static const nrfx_saadc_channel_t s_channels[] = {
    SAADC_CHANNEL_SE_ACQ(CH0_AIN, NULL, 0, 1),
    SAADC_CHANNEL_SE_ACQ(CH1_AIN, NULL, 1, 1),
};
#define SA_CH_COUNT NRFX_ARRAY_SIZE(s_channels)

static uint16_t s_sa_buf[2][CHUNK_SAMPLES * 2];

static nrfx_timer_t s_timer = NRFX_TIMER_INSTANCE(0);
#define DPPI_CH 0

static inline void gen_sine(uint16_t n, uint16_t amp, nrf_pwm_values_individual_t *dst) {
    if (n == 0) return;
    const float offset = PWM_PERIOD / 2.0f;
    for (uint16_t i = 0; i < n; i++) {
        float phase = 2.0f * (float)PI * ((float)i) / (float)n;
        int32_t v = (int32_t)((amp/2.0f) * sinf(phase) + offset);
        if (v < 0) v = 0;
        if (v > PWM_PERIOD) v = PWM_PERIOD;
        dst[i].channel_0 = (uint16_t)v;
    }
}

static inline uint16_t step_from_err(int32_t err) {
    int32_t a = (err < 0) ? -err : err;
    if (a > 200) return 10;
    if (a > 100) return 5;
    if (a > 50) return 2;
    if (a > 20)  return 1;
    return 0;
}

void adc_pwm_set_freqs(uint16_t f0_hz, uint16_t f1_hz)
{
    if (f0_hz == 0) f0_hz = g_freq0_hz;
    if (f1_hz == 0) f1_hz = g_freq1_hz;

    uint16_t n0 = (uint16_t)((100000U + (f0_hz/2)) / f0_hz);
    uint16_t n1 = (uint16_t)((100000U + (f1_hz/2)) / f1_hz);
    if (n0 < 10) {n0 = 10;} if (n0 > MAX_SAMPLES) n0 = MAX_SAMPLES;
    if (n1 < 10) {n1 = 10;} if (n1 > MAX_SAMPLES) n1 = MAX_SAMPLES;

    g_freq0_hz = f0_hz; g_nsamp0 = n0;
    g_freq1_hz = f1_hz; g_nsamp1 = n1;

    gen_sine(g_nsamp0, g_amp0, s_vals0);
    gen_sine(g_nsamp1, g_amp1, s_vals1);
    nrf_pwm_sequence_t s_seq0 = { .values.p_individual = s_vals0, .length = 4 * g_nsamp0, .repeats = 0, .end_delay = 0 };
    nrf_pwm_sequence_t s_seq1 = { .values.p_individual = s_vals1, .length = 4 * g_nsamp1, .repeats = 0, .end_delay = 0 };

    if (g_started) {
        (void)nrfx_pwm_simple_playback(&s_pwm0, &s_seq0, 1, NRFX_PWM_FLAG_LOOP);
        (void)nrfx_pwm_simple_playback(&s_pwm1, &s_seq1, 1, NRFX_PWM_FLAG_LOOP);
    } else {
        nrfx_pwm_stop(&s_pwm0, true);
        nrfx_pwm_stop(&s_pwm1, true);
    }
}

static void timer_route(uint32_t fs_hz) {
    nrfx_timer_config_t tcfg = (nrfx_timer_config_t){
        .frequency = NRF_TIMER_FREQ_1MHz,
        .mode      = NRF_TIMER_MODE_TIMER,
        .bit_width = NRF_TIMER_BIT_WIDTH_32,
        .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
        .p_context = NULL,
    };
    nrfx_timer_init(&s_timer, &tcfg, NULL);

    if (fs_hz == 0) fs_hz = 1;
    uint32_t cc = 1000000UL / fs_hz;
    nrfx_timer_clear(&s_timer);
    nrfx_timer_extended_compare(&s_timer, NRF_TIMER_CC_CHANNEL0, cc, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

    nrf_timer_publish_set(s_timer.p_reg, NRF_TIMER_EVENT_COMPARE0, DPPI_CH);
    nrf_saadc_subscribe_set(NRF_SAADC, NRF_SAADC_TASK_SAMPLE, DPPI_CH);
    nrf_dppi_channels_enable(NRF_DPPIC, BIT(DPPI_CH));
    nrfx_timer_enable(&s_timer);
}

static volatile bool g_amp_changed0 = false;
static volatile bool g_amp_changed1 = false;
static void saadc_handler(nrfx_saadc_evt_t const *evt) {
    switch (evt->type) {
    case NRFX_SAADC_EVT_CALIBRATEDONE:
        (void)nrfx_saadc_mode_trigger();
        break;
    case NRFX_SAADC_EVT_BUF_REQ:
        (void)nrfx_saadc_buffer_set(s_sa_buf[1], CHUNK_SAMPLES * 2);
        break;
    case NRFX_SAADC_EVT_DONE: {
        if (!g_started) {
            break;
        }

        uint16_t *buf = evt->data.done.p_buffer;
        uint16_t sz   = evt->data.done.size;

        uint16_t max0 = 0, min0 = 0xFFFF;
        uint16_t max1 = 0, min1 = 0xFFFF;

        uint16_t pairs = sz / 2;
        if (pairs > CHUNK_SAMPLES) pairs = CHUNK_SAMPLES;
        for (uint16_t i = 0; i < pairs; i++) {
            uint16_t s0 = buf[(i << 1) + 0];
            uint16_t s1 = buf[(i << 1) + 1];
            if (s0 > max0) max0 = s0;
            if (s0 < min0) min0 = s0;
            if (s1 > max1) max1 = s1;
            if (s1 < min1) min1 = s1;
        }
        uint16_t p2p0 = (max0 > min0) ? (max0 - min0) : 0;
        uint16_t p2p1 = (max1 > min1) ? (max1 - min1) : 0;
        g_last_p2p0 = p2p0;
        g_last_p2p1 = p2p1;
        
        if (g_started) {
            int32_t err0 = (int32_t)p2p0 - (int32_t)g_th0;
            int32_t err1 = (int32_t)p2p1 - (int32_t)g_th1;
            uint16_t st0 = step_from_err(err0);
            uint16_t st1 = step_from_err(err1);
            if (st0) {
                if (err0>0) { if (g_amp0 > st0) g_amp0 -= st0; }
                else         { if (g_amp0 + st0 < PWM_PERIOD) g_amp0 += st0; }
                g_amp_changed0 = true;
            }
            if (st1) {
                if (err1>0) { if (g_amp1 > st1) g_amp1 -= st1; }
                else         { if (g_amp1 + st1 < PWM_PERIOD) g_amp1 += st1; }
                g_amp_changed1 = true;
            }

            if (g_amp_changed0) {
                g_amp_changed0 = false;
                gen_sine(g_nsamp0, g_amp0, s_vals0);
                nrf_pwm_sequence_t s_seq0 = { .values.p_individual = s_vals0, .length = 4 * g_nsamp0, .repeats = 0, .end_delay = 0 };
                (void)nrfx_pwm_simple_playback(&s_pwm0, &s_seq0, 1, NRFX_PWM_FLAG_LOOP);
            }
            if (g_amp_changed1) {
                g_amp_changed1 = false;
                gen_sine(g_nsamp1, g_amp1, s_vals1);
                nrf_pwm_sequence_t s_seq1 = { .values.p_individual = s_vals1, .length = 4 * g_nsamp1, .repeats = 0, .end_delay = 0 };
                (void)nrfx_pwm_simple_playback(&s_pwm1, &s_seq1, 1, NRFX_PWM_FLAG_LOOP);
            }
        }

        if (ble_notify_enabled()) {
            (void)ble_notify_status(p2p0, p2p1,
                                    adc_pwm_get_amp0_code(),
                                    adc_pwm_get_amp1_code());
        }
        break;
    }
    default:
        break;
    }
}

void adc_pwm_set_start(bool on) { g_started = on; }
void adc_pwm_set_targets(uint16_t th0, uint16_t th1) { g_th0 = th0; g_th1 = th1; }
void adc_pwm_set_mux(uint8_t mux) { g_mux = mux & 0x01; }
uint8_t  adc_pwm_get_amp0_code(void) { uint16_t a=g_amp0; if(a>PWM_PERIOD)a=PWM_PERIOD; return (uint8_t)a; }
uint8_t  adc_pwm_get_amp1_code(void) { uint16_t a=g_amp1; if(a>PWM_PERIOD)a=PWM_PERIOD; return (uint8_t)a; }
uint16_t adc_pwm_get_last_p2p_ch0(void) { return g_last_p2p0; }
uint16_t adc_pwm_get_last_p2p_ch1(void) { return g_last_p2p1; }

void adc_pwm_init(void) {
#if defined(__ZEPHYR__)
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_PWM_INST_GET(PWM_INST_IDX0)), IRQ_PRIO_LOWEST,
                NRFX_PWM_INST_HANDLER_GET(PWM_INST_IDX0), 0, 0);
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_PWM_INST_GET(PWM_INST_IDX1)), IRQ_PRIO_LOWEST,
                NRFX_PWM_INST_HANDLER_GET(PWM_INST_IDX1), 0, 0);
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SAADC), IRQ_PRIO_LOWEST, nrfx_saadc_irq_handler, 0, 0);
#endif

    nrfx_pwm_config_t cfg0 = NRFX_PWM_DEFAULT_CONFIG(LED1_PIN, NULL, NULL, NULL);
    cfg0.top_value = PWM_PERIOD;
    cfg0.base_clock = NRF_PWM_CLK_16MHz;
    cfg0.load_mode = NRF_PWM_LOAD_INDIVIDUAL;
    (void)nrfx_pwm_init(&s_pwm0, &cfg0, NULL, &s_pwm0);

    nrfx_pwm_config_t cfg1 = NRFX_PWM_DEFAULT_CONFIG(LED2_PIN, NULL, NULL, NULL);
    cfg1.top_value = PWM_PERIOD;
    cfg1.base_clock = NRF_PWM_CLK_16MHz;
    cfg1.load_mode = NRF_PWM_LOAD_INDIVIDUAL;
    (void)nrfx_pwm_init(&s_pwm1, &cfg1, NULL, &s_pwm1);

    gen_sine(NUM_SAMPLES0, g_amp0, s_vals0);
    gen_sine(NUM_SAMPLES1, g_amp1, s_vals1);
    nrf_pwm_sequence_t s_seq0 = { .values.p_individual = s_vals0, .length = 4 * g_nsamp0, .repeats = 0, .end_delay = 0 };
    nrf_pwm_sequence_t s_seq1 = { .values.p_individual = s_vals1, .length = 4 * g_nsamp1, .repeats = 0, .end_delay = 0 };
    if (g_started) {
        (void)nrfx_pwm_simple_playback(&s_pwm0, &s_seq0, 1, NRFX_PWM_FLAG_LOOP);
        (void)nrfx_pwm_simple_playback(&s_pwm1, &s_seq1, 1, NRFX_PWM_FLAG_LOOP);
    } else {
        nrfx_pwm_stop(&s_pwm0, true);
        nrfx_pwm_stop(&s_pwm1, true);
    }

    (void)nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
    (void)nrfx_saadc_channels_config(s_channels, SA_CH_COUNT);

    nrfx_saadc_adv_config_t adv = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    adv.internal_timer_cc = 0;
    adv.start_on_end = true;
    uint32_t mask = nrfx_saadc_channels_configured_get();
    (void)nrfx_saadc_advanced_mode_set(mask, ADC_RESOLUTION, &adv, saadc_handler);

    (void)nrfx_saadc_buffer_set(s_sa_buf[0], CHUNK_SAMPLES * 2);
    (void)nrfx_saadc_buffer_set(s_sa_buf[1], CHUNK_SAMPLES * 2);

    timer_route(SAADC_SAMPLE_FREQUENCY);

    (void)nrfx_saadc_offset_calibrate(saadc_handler);

    nrf_gpio_cfg_output(GPIO_PIN0);
    nrf_gpio_cfg_output(GPIO_PIN1);
    nrf_gpio_cfg_output(GPIO_PWR);
    
    nrf_gpio_pin_clear(GPIO_PIN0);
    nrf_gpio_pin_clear(GPIO_PIN1);
    nrf_gpio_pin_clear(GPIO_PWR);
    
    while (1) {
        if (g_mux == 0) { nrf_gpio_pin_clear(GPIO_PIN0); nrf_gpio_pin_clear(GPIO_PIN1); }
        else            { nrf_gpio_pin_set(GPIO_PIN0); nrf_gpio_pin_set(GPIO_PIN1); }

        nrf_gpio_pin_set(GPIO_PWR);
        k_sleep(K_MSEC(1));
        nrf_gpio_pin_clear(GPIO_PWR);
        k_sleep(K_MSEC(10));
    }
}

