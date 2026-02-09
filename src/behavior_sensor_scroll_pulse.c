/*
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT zmk_behavior_sensor_scroll_pulse

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/virtual_key_position.h>
#include <zmk/events/position_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#ifndef ZMK_KEYMAP_SENSORS_LEN
#define ZMK_KEYMAP_SENSORS_LEN 0
#endif

#ifndef ZMK_KEYMAP_LAYERS_LEN
#define ZMK_KEYMAP_LAYERS_LEN 1
#endif

// ZMK本体(behavior_input_two_axis.c)にある関数を利用
extern int behavior_input_two_axis_adjust_speed(const struct device *dev, int16_t dx, int16_t dy);

enum dir {
    DIR_NONE = 0,
    DIR_CW = 1,
    DIR_CCW = 2,
};

enum axis_mode {
    AXIS_WHEEL = 0,   // vertical
    AXIS_HWHEEL = 1,  // horizontal
};

struct behavior_sensor_scroll_pulse_config {
    const char *target_behavior; // phandle "binding" の device name

    uint16_t pulse_ms;           // これだけ“押しっぱなし”にする
    uint16_t fast_dt_ms;         // これ以下なら max_speed
    uint16_t slow_dt_ms;         // これ以上なら min_speed
    int16_t  min_speed;          // adjust_speed に渡す量
    int16_t  max_speed;
    uint8_t  axis;               // 0 wheel / 1 hwheel
};

struct slot_state {
    // 直近の方向 (accept_dataでセット)
    enum dir pending_dir;

    // dt計算用
    int64_t last_ts;

    // 現在“押しっぱなし”にしてる合計量（連続回転で積み増し）
    int16_t acc_dx;
    int16_t acc_dy;

    struct k_work_delayable release_work;

    // コールバック内で target を叩くために保存
    const struct device *parent_dev;
    uint8_t sensor_index;
    uint8_t layer;
};

struct behavior_sensor_scroll_pulse_data {
    struct slot_state st[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
};

static inline int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return (int16_t)v;
}

static inline int get_sensor_index_safe(uint32_t pos) {
    int si = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(pos);
    if (si >= 0 && si < ZMK_KEYMAP_SENSORS_LEN) return si;

    if ((int)pos >= 0 && (int)pos < ZMK_KEYMAP_SENSORS_LEN) return (int)pos;
    return -1;
}

static int16_t dt_to_speed(const struct behavior_sensor_scroll_pulse_config *cfg, uint16_t dt_ms) {
    // dt が短いほど speed 大
    if (cfg->fast_dt_ms == 0 || dt_ms <= cfg->fast_dt_ms) return cfg->max_speed;
    if (cfg->slow_dt_ms == 0 || dt_ms >= cfg->slow_dt_ms) return cfg->min_speed;

    // linear interpolate: [fast..slow] -> [max..min]
    int32_t num = (int32_t)(dt_ms - cfg->fast_dt_ms) * (cfg->min_speed - cfg->max_speed);
    int32_t den = (int32_t)(cfg->slow_dt_ms - cfg->fast_dt_ms);
    int32_t out = (int32_t)cfg->max_speed + (num / den);

    return clamp_i16(out, cfg->min_speed, cfg->max_speed);
}

static void release_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct slot_state *st = CONTAINER_OF(dwork, struct slot_state, release_work);

    const struct device *dev = st->parent_dev;
    if (!dev) return;

    const struct behavior_sensor_scroll_pulse_config *cfg = dev->config;
    const struct device *target = zmk_behavior_get_binding(cfg->target_behavior);
    if (!target) return;

    if (st->acc_dx != 0 || st->acc_dy != 0) {
        // “押しっぱなし”分を戻して停止
        behavior_input_two_axis_adjust_speed(target, (int16_t)-st->acc_dx, (int16_t)-st->acc_dy);
        st->acc_dx = 0;
        st->acc_dy = 0;
    }
}

static int accept_data(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event,
                       const struct zmk_sensor_config *sensor_config,
                       size_t channel_data_size,
                       const struct zmk_sensor_channel_data *channel_data) {
    ARG_UNUSED(binding);
    ARG_UNUSED(sensor_config);
    ARG_UNUSED(channel_data_size);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_sensor_scroll_pulse_data *data = dev->data;

    int sensor_index = get_sensor_index_safe(event.position);
    if (sensor_index < 0 || event.layer >= ZMK_KEYMAP_LAYERS_LEN) {
        return 0;
    }

    const struct sensor_value v = channel_data[0].value;
    int delta = (v.val1 == 0) ? v.val2 : v.val1;

    enum dir d = DIR_NONE;
    if (delta > 0) d = DIR_CW;
    else if (delta < 0) d = DIR_CCW;

    data->st[sensor_index][event.layer].pending_dir = d;

    return 0;
}

static int process(struct zmk_behavior_binding *binding,
                   struct zmk_behavior_binding_event event,
                   enum behavior_sensor_binding_process_mode mode) {

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_sensor_scroll_pulse_config *cfg = dev->config;
    struct behavior_sensor_scroll_pulse_data *data = dev->data;

    int sensor_index = get_sensor_index_safe(event.position);
    if (sensor_index < 0 || event.layer >= ZMK_KEYMAP_LAYERS_LEN) {
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    struct slot_state *st = &data->st[sensor_index][event.layer];

    if (mode != BEHAVIOR_SENSOR_BINDING_PROCESS_MODE_TRIGGER) {
        st->pending_dir = DIR_NONE;
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    enum dir d = st->pending_dir;
    st->pending_dir = DIR_NONE;
    if (d == DIR_NONE) return ZMK_BEHAVIOR_TRANSPARENT;

#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    event.source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL;
#endif

    // target device
    const struct device *target = zmk_behavior_get_binding(cfg->target_behavior);
    if (!target) return ZMK_BEHAVIOR_TRANSPARENT;

    int64_t now = k_uptime_get();
    uint16_t dt_ms = 9999;
    if (st->last_ts != 0 && now > st->last_ts) {
        int64_t dt = now - st->last_ts;
        if (dt < 0) dt = 0;
        if (dt > 65535) dt = 65535;
        dt_ms = (uint16_t)dt;
    }
    st->last_ts = now;

    int16_t sp = dt_to_speed(cfg, dt_ms);
    int16_t sign = (d == DIR_CW) ? 1 : -1;

    int16_t dx = 0, dy = 0;
    if (cfg->axis == AXIS_HWHEEL) {
        dx = (int16_t)(sign * sp);
    } else {
        dy = (int16_t)(sign * sp);
    }

    // “短時間ホールド”として速度を積み増し
    behavior_input_two_axis_adjust_speed(target, dx, dy);
    st->acc_dx += dx;
    st->acc_dy += dy;

    // release work を延長
    st->parent_dev = dev;
    st->sensor_index = (uint8_t)sensor_index;
    st->layer = (uint8_t)event.layer;

    uint16_t pulse = cfg->pulse_ms ? cfg->pulse_ms : 30;
    k_work_reschedule(&st->release_work, K_MSEC(pulse));

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api api = {
    .sensor_binding_accept_data = accept_data,
    .sensor_binding_process = process,
};

static int init(const struct device *dev) {
    struct behavior_sensor_scroll_pulse_data *data = dev->data;

    for (int s = 0; s < ZMK_KEYMAP_SENSORS_LEN; s++) {
        for (int l = 0; l < ZMK_KEYMAP_LAYERS_LEN; l++) {
            struct slot_state *st = &data->st[s][l];
            st->pending_dir = DIR_NONE;
            st->last_ts = 0;
            st->acc_dx = 0;
            st->acc_dy = 0;
            st->parent_dev = dev;
            st->sensor_index = (uint8_t)s;
            st->layer = (uint8_t)l;
            k_work_init_delayable(&st->release_work, release_work_cb);
        }
    }
    return 0;
}

#define INST(n)                                                                                     \
    static struct behavior_sensor_scroll_pulse_data data_##n = {};                                  \
    static const struct behavior_sensor_scroll_pulse_config cfg_##n = {                             \
        .target_behavior = DT_LABEL(DT_INST_PHANDLE(n, binding)),                         \
        .pulse_ms   = DT_INST_PROP_OR(n, pulse_ms, 30),                                             \
        .fast_dt_ms = DT_INST_PROP_OR(n, fast_dt_ms, 60),                                           \
        .slow_dt_ms = DT_INST_PROP_OR(n, slow_dt_ms, 250),                                          \
        .min_speed  = DT_INST_PROP_OR(n, min_speed, 3),                                             \
        .max_speed  = DT_INST_PROP_OR(n, max_speed, 18),                                            \
        .axis       = DT_INST_PROP_OR(n, axis, 0),                                                  \
    };                                                                                              \
    BEHAVIOR_DT_INST_DEFINE(n, init, NULL, &data_##n, &cfg_##n,                                      \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api);

DT_INST_FOREACH_STATUS_OKAY(INST)
