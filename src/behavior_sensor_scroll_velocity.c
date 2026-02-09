/*
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT zmk_behavior_sensor_scroll_velocity

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/virtual_key_position.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#ifndef ZMK_KEYMAP_SENSORS_LEN
#define ZMK_KEYMAP_SENSORS_LEN 0
#endif
#ifndef ZMK_KEYMAP_LAYERS_LEN
#define ZMK_KEYMAP_LAYERS_LEN 1
#endif

enum scroll_dir {
    SCROLL_NONE = 0,
    SCROLL_CW = 1,
    SCROLL_CCW = 2,
};

struct behavior_sensor_scroll_velocity_config {
    uint8_t axis;   // 0 vertical, 1 horizontal
    uint8_t invert; // 0/1

    int16_t slow_amount;
    int16_t fast_amount;

    uint16_t fast_threshold_ms;
    uint16_t slow_threshold_ms;

    uint16_t ema_alpha_permil; // 0..1000
};

struct per_state {
    bool inited;
    uint32_t last_ts_ms;
    uint32_t ema_dt_ms;
};

struct behavior_sensor_scroll_velocity_data {
    enum scroll_dir pending_dir[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
    struct per_state st[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
};

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static uint32_t ema_u32(uint32_t prev, uint32_t cur, uint16_t alpha_permil) {
    if (alpha_permil >= 1000) return cur;
    if (alpha_permil == 0) return prev;
    uint32_t a = alpha_permil;
    uint32_t b = 1000 - a;
    return (prev * b + cur * a) / 1000;
}

static int16_t map_dt_to_amount(const struct behavior_sensor_scroll_velocity_config *cfg,
                                uint32_t dt_ms) {
    uint32_t fast_ms = cfg->fast_threshold_ms ? cfg->fast_threshold_ms : 20;
    uint32_t slow_ms = cfg->slow_threshold_ms ? cfg->slow_threshold_ms : 140;
    if (slow_ms < fast_ms + 1) slow_ms = fast_ms + 1;

    dt_ms = clamp_u32(dt_ms, fast_ms, slow_ms);

    int32_t slow_amt = cfg->slow_amount ? cfg->slow_amount : 1;
    int32_t fast_amt = cfg->fast_amount ? cfg->fast_amount : 6;

    // dt small => fast => fast_amt
    int32_t num = (int32_t)(dt_ms - fast_ms) * (slow_amt - fast_amt);
    int32_t den = (int32_t)(slow_ms - fast_ms);
    int32_t amt = fast_amt + (den ? (num / den) : 0);

    if (amt < 1) amt = 1;
    // clamp to [min(slow,fast), max(slow,fast)]
    int32_t lo = (slow_amt < fast_amt) ? slow_amt : fast_amt;
    int32_t hi = (slow_amt > fast_amt) ? slow_amt : fast_amt;
    if (amt < lo) amt = lo;
    if (amt > hi) amt = hi;

    return (int16_t)amt;
}

static inline bool in_range(int idx, int max) { return idx >= 0 && idx < max; }

static int accept_data(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event,
                       const struct zmk_sensor_config *sensor_config,
                       size_t channel_data_size,
                       const struct zmk_sensor_channel_data *channel_data) {
    ARG_UNUSED(sensor_config);
    ARG_UNUSED(channel_data_size);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_sensor_scroll_velocity_data *data = dev->data;

    const int sensor_index = (int)event.position;   // ★ まずはそのまま使う
    const int layer = (int)event.layer;

    // ★★★絶対にメモリ破壊しないためのガード
    if (!in_range(sensor_index, ZMK_KEYMAP_SENSORS_LEN) || !in_range(layer, ZMK_KEYMAP_LAYERS_LEN)) {
        return 0;
    }

    const struct sensor_value v = channel_data[0].value;
    int delta = (v.val1 == 0) ? v.val2 : v.val1;

    if (delta > 0) {
        data->pending_dir[sensor_index][layer] = SCROLL_CW;
    } else if (delta < 0) {
        data->pending_dir[sensor_index][layer] = SCROLL_CCW;
    } else {
        data->pending_dir[sensor_index][layer] = SCROLL_NONE;
    }

    return 0;
}

static int process(struct zmk_behavior_binding *binding,
                   struct zmk_behavior_binding_event event,
                   enum behavior_sensor_binding_process_mode mode) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_sensor_scroll_velocity_config *cfg = dev->config;
    struct behavior_sensor_scroll_velocity_data *data = dev->data;

    const int sensor_index = (int)event.position;  // ★そのまま
    const int layer = (int)event.layer;

    if (!in_range(sensor_index, ZMK_KEYMAP_SENSORS_LEN) || !in_range(layer, ZMK_KEYMAP_LAYERS_LEN)) {
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    if (mode != BEHAVIOR_SENSOR_BINDING_PROCESS_MODE_TRIGGER) {
        data->pending_dir[sensor_index][layer] = SCROLL_NONE;
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    enum scroll_dir dir = data->pending_dir[sensor_index][layer];
    data->pending_dir[sensor_index][layer] = SCROLL_NONE;

    if (dir == SCROLL_NONE) return ZMK_BEHAVIOR_TRANSPARENT;

    struct per_state *st = &data->st[sensor_index][layer];

    uint32_t now_ms = (uint32_t)k_uptime_get();
    if (!st->inited) {
        st->inited = true;
        st->last_ts_ms = now_ms;
        st->ema_dt_ms = cfg->slow_threshold_ms ? cfg->slow_threshold_ms : 140;
    }

    uint32_t dt = now_ms - st->last_ts_ms;
    st->last_ts_ms = now_ms;

    st->ema_dt_ms = ema_u32(st->ema_dt_ms, dt, cfg->ema_alpha_permil);

    int16_t amt = map_dt_to_amount(cfg, st->ema_dt_ms);

    int sign = (dir == SCROLL_CW) ? 1 : -1;
    if (cfg->invert) sign = -sign;

    int16_t wheel = (int16_t)(sign * amt);
    uint16_t code = (cfg->axis == 1) ? INPUT_REL_HWHEEL : INPUT_REL_WHEEL;

    input_report_rel(dev, code, wheel, true, K_NO_WAIT);
    return ZMK_BEHAVIOR_OPAQUE;
}


static const struct behavior_driver_api api = {
    .sensor_binding_accept_data = accept_data,
    .sensor_binding_process = process,
};

#define INST(n)                                                                                \
    static const struct behavior_sensor_scroll_velocity_config cfg_##n = {                     \
        .axis = DT_INST_PROP_OR(n, axis, 0),                                                   \
        .invert = DT_INST_PROP_OR(n, invert, 0),                                               \
        .slow_amount = DT_INST_PROP_OR(n, slow_amount, 1),                                     \
        .fast_amount = DT_INST_PROP_OR(n, fast_amount, 6),                                     \
        .fast_threshold_ms = DT_INST_PROP_OR(n, fast_threshold_ms, 20),                         \
        .slow_threshold_ms = DT_INST_PROP_OR(n, slow_threshold_ms, 140),                        \
        .ema_alpha_permil = DT_INST_PROP_OR(n, ema_alpha_permil, 300),                          \
    };                                                                                         \
    static struct behavior_sensor_scroll_velocity_data data_##n = {};                          \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, &data_##n, &cfg_##n,                                \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api);

DT_INST_FOREACH_STATUS_OKAY(INST)
