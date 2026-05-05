#include "parameter.h"
#include "adc_pwm.h"
#include "ble_service.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>

#if Device == 1
static struct bt_uuid_128 SVC_UUID  = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4F191C00,0x5A23,0x4A82,0x8F3D,0x6C9E4E9B0000ULL)
);
static struct bt_uuid_128 TX_CHR_UUID= BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4F191C01,0x5A23,0x4A82,0x8F3D,0x6C9E4E9B0000ULL)
);
static struct bt_uuid_128 RX_CHR_UUID= BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4F191C02,0x5A23,0x4A82,0x8F3D,0x6C9E4E9B0000ULL)
);
#endif

static struct bt_conn *g_conn;
static atomic_t g_ccc_on = ATOMIC_INIT(0);

static ssize_t rd_fn(struct bt_conn *c, const struct bt_gatt_attr *a,
                     void *buf, uint16_t len, uint16_t off)
{
    uint8_t pkt[6];
    uint16_t p2p0 = adc_pwm_get_last_p2p_ch0();
    uint16_t p2p1 = adc_pwm_get_last_p2p_ch1();

    pkt[0] = (uint8_t)(p2p0 & 0xFF);
    pkt[1] = (uint8_t)((p2p0 >> 8) & 0xFF);
    pkt[2] = (uint8_t)(p2p1 & 0xFF);
    pkt[3] = (uint8_t)((p2p1 >> 8) & 0xFF);
    pkt[4] = adc_pwm_get_amp0_code();
    pkt[5] = adc_pwm_get_amp1_code();
    return bt_gatt_attr_read(c, a, buf, len, off, pkt, sizeof(pkt));
}

static ssize_t wr_fn(struct bt_conn *c, const struct bt_gatt_attr *a,
                     const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (offset != 0 || len < 6)
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);

    const uint8_t *p = (const uint8_t *)buf;
    bool start = (p[0] & 0x10) ? true : false;
    uint16_t th0 = (uint16_t)(p[1] | (p[2] << 8));
    uint16_t th1 = (uint16_t)(p[3] | (p[4] << 8));
    uint8_t mux   = p[5] & 0x01;

    adc_pwm_set_targets(th0, th1);
    adc_pwm_set_start(start);
    adc_pwm_set_mux(mux);

    if (mux == 0) { nrf_gpio_pin_clear(GPIO_PIN0); nrf_gpio_pin_clear(GPIO_PIN1); }
    else          { nrf_gpio_pin_set(GPIO_PIN0); nrf_gpio_pin_set(GPIO_PIN1); }

    if (len >= 10) {
        uint16_t f0 = (uint16_t)(p[6] | (p[7] << 8));
        uint16_t f1 = (uint16_t)(p[8] | (p[9] << 8));
        if (f0) {
            adc_pwm_set_freqs(f0, f1);
        }
    }

    return len;
}

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    bool on = (value & BT_GATT_CCC_NOTIFY) != 0;
    atomic_set(&g_ccc_on, on);
}

BT_GATT_SERVICE_DEFINE(tis_svc,
    BT_GATT_PRIMARY_SERVICE(&SVC_UUID),

    BT_GATT_CHARACTERISTIC(&RX_CHR_UUID.uuid,
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE, NULL, wr_fn, NULL),

    BT_GATT_CHARACTERISTIC(&TX_CHR_UUID.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ, rd_fn, NULL, NULL),
    BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

static const struct bt_gatt_attr *rd_attr_ptr(void)
{
    return &tis_svc.attrs[4];
}

static int adv_start_safe(void);

static void connected(struct bt_conn *c, uint8_t err)
{
    if (err) return;
    g_conn = bt_conn_ref(c);

    (void)bt_conn_le_param_update(c, &(struct bt_le_conn_param){
        .interval_min = REQ_CONN_INTERVAL_MIN,
        .interval_max = REQ_CONN_INTERVAL_MAX,
        .latency      = REQ_CONN_LATENCY,
        .timeout      = REQ_CONN_TIMEOUT,
    });
}

static void disconnected(struct bt_conn *c, uint8_t reason)
{
    if (g_conn) { bt_conn_unref(g_conn); g_conn = NULL; }
    atomic_set(&g_ccc_on, 0);
    (void)adv_start_safe();
}

BT_CONN_CB_DEFINE(conn_cb) = {
    .connected = connected,
    .disconnected = disconnected
};

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_UUID128_ALL, SVC_UUID.val, sizeof(SVC_UUID.val))
};
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, APP_DEVICE_NAME, strlen(APP_DEVICE_NAME)),
};

static int adv_start_safe(void)
{
    int err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err == -EALREADY) {
        (void)bt_le_adv_stop();
        err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    }
    return err;
}

static void bt_ready_cb(int err)
{
    if (err) return;
#if defined(CONFIG_SETTINGS)
    settings_load();
#endif
    (void)adv_start_safe();
}

int ble_init_and_advertise(void)
{
    (void)bt_set_name(APP_DEVICE_NAME);
    int rc = bt_enable(bt_ready_cb);
    if (rc == -EALREADY) (void)adv_start_safe();
    return (rc && rc != -EALREADY) ? rc : 0;
}

bool ble_notify_enabled(void)
{
    return (g_conn && atomic_get(&g_ccc_on));
}

int ble_notify_status(uint8_t p2p0, uint16_t p2p1, uint8_t amp0, uint8_t amp1)
{
    if (!ble_notify_enabled()) return -EACCES;
    uint8_t pkt[6] = {
        (uint8_t)(p2p0 & 0xFF), (uint8_t)((p2p0 >> 8) & 0xFF),
        (uint8_t)(p2p1 & 0xFF), (uint8_t)((p2p1 >> 8) & 0xFF),
        amp0, amp1
    };
    struct bt_gatt_notify_params np = {
        .attr = rd_attr_ptr(),
        .data = pkt,
        .len  = sizeof(pkt),
    };
    return bt_gatt_notify_cb(g_conn, &np);
}