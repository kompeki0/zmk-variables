/*
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT zmk_behavior_sensor_rotate_accel

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h> // CLAMP

#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <zmk/behavior_queue.h>
#include <zmk/virtual_key_position.h>

#include <dt-bindings/zmk/pointing.h> // MOVE(), MOVE_X_DECODE, MOVE_Y_DECODE

#ifndef ZMK_KEYMAP_SENSORS_LEN
#define ZMK_KEYMAP_SENSORS_LEN 0
#endif

#ifndef ZMK_KEYMAP_LAYERS_LEN
#define ZMK_KEYMAP_LAYERS_LEN 1
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

enum accel_dir {
    ACCEL_DIR_NONE = 0,
    ACCEL_DIR_CW = 1,
    ACCEL_DIR_CCW = 2,
};

struct behavior_sensor_rotate_accel_config {
    struct zmk_behavior_binding cw_binding;
    struct zmk_behavior_binding ccw_binding;

    uint16_t dt_min_ms;
    uint16_t dt_max_ms;

    uint8_t steps_min;
    uint8_t steps_max;

    uint8_t curve;   // 0 linear, 1 quad, 2 cubic
    bool same_dir_only;

    uint16_t tap_ms;
};

struct behavior_sensor_rotate_accel_data {
    enum accel_dir pending_dir[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];

    uint32_t last_ms[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
    enum accel_dir last_dir[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
};

static inline int enqueue_press(struct zmk_behavior_binding_event *event,
                                struct zmk_behavior_binding binding, uint16_t wait_ms) {
    return zmk_behavior_queue_add(event, binding, true, wait_ms);
}
static inline int enqueue_release(struct zmk_behavior_binding_event *event,
                                  struct zmk_behavior_binding binding) {
    return zmk_behavior_queue_add(event, binding, false, 0);
}

static uint8_t clamp_steps(uint8_t v, uint8_t smin, uint8_t smax) {
    if (smax < smin) {
        uint8_t t = smin; smin = smax; smax = t;
    }
    if (v < smin) return smin;
    if (v > smax) return smax;
    return v;
}

static uint8_t calc_steps_linear(uint32_t dt, uint32_t dt_min, uint32_t dt_max,
                                 uint8_t smin, uint8_t smax) {
    if (smax < smin) { uint8_t t=smin; smin=smax; smax=t; }
    if (dt <= dt_min) return smax;
    if (dt >= dt_max) return smin;

    uint32_t den = (dt_max - dt_min);
    uint32_t num = (dt_max - dt) * (uint32_t)(smax - smin);
    return (uint8_t)(smin + (num / den));
}

static uint8_t calc_steps_quad(uint32_t dt, uint32_t dt_min, uint32_t dt_max,
                               uint8_t smin, uint8_t smax) {
    if (smax < smin) { uint8_t t=smin; smin=smax; smax=t; }
    if (dt <= dt_min) return smax;
    if (dt >= dt_max) return smin;

    uint32_t den = (dt_max - dt_min);
    uint32_t num = (dt_max - dt);

    // t in Q15
    uint32_t t_q15 = (num << 15) / den;
    // t^2 in Q15
    uint32_t t2_q15 = (t_q15 * t_q15) >> 15;

    uint32_t range = (uint32_t)(smax - smin);
    uint32_t add = (range * t2_q15) >> 15;
    return (uint8_t)(smin + add);
}

static uint8_t calc_steps_cubic(uint32_t dt, uint32_t dt_min, uint32_t dt_max,
                                uint8_t smin, uint8_t smax) {
    if (smax < smin) { uint8_t t=smin; smin=smax; smax=t; }
    if (dt <= dt_min) return smax;
    if (dt >= dt_max) return smin;

    uint32_t den = (dt_max - dt_min);
    uint32_t num = (dt_max - dt);

    uint32_t t_q15 = (num << 15) / den;
    uint32_t t2_q15 = (t_q15 * t_q15) >> 15;
    uint32_t t3_q15 = (t2_q15 * t_q15) >> 15;

    uint32_t range = (uint32_t)(smax - smin);
    uint32_t add = (range * t3_q15) >> 15;
    return (uint8_t)(smin + add);
}

static uint8_t calc_steps(const struct behavior_sensor_rotate_accel_config *cfg, uint32_t dt) {
    uint32_t dt_min = cfg->dt_min_ms;
    uint32_t dt_max = cfg->dt_max_ms;
    if (dt_min < 1) dt_min = 1;
    if (dt_max <= dt_min) dt_max = dt_min + 1;

    uint8_t smin = cfg->steps_min ? cfg->steps_min : 1;
    uint8_t smax = cfg->steps_max ? cfg->steps_max : 5;

    uint8_t s;
    switch (cfg->curve) {
    case 0:
        s = calc_steps_linear(dt, dt_min, dt_max, smin, smax);
        break;
    case 2:
        s = calc_steps_cubic(dt, dt_min, dt_max, smin, smax);
        break;
    case 1:
    default:
        s = calc_steps_quad(dt, dt_min, dt_max, smin, smax);
        break;
    }
    return clamp_steps(s, smin, smax);
}

static struct zmk_behavior_binding scale_move_binding(struct zmk_behavior_binding b, uint8_t steps) {
    // param1 は MOVE(x,y) 想定
    int16_t x = MOVE_X_DECODE(b.param1);
    int16_t y = MOVE_Y_DECODE(b.param1);

    int32_t sx = (int32_t)x * (int32_t)steps;
    int32_t sy = (int32_t)y * (int32_t)steps;

    sx = CLAMP(sx, INT16_MIN, INT16_MAX);
    sy = CLAMP(sy, INT16_MIN, INT16_MAX);

    b.param1 = MOVE((int16_t)sx, (int16_t)sy);
    return b;
}

static int accept_data(struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event,
                       const struct zmk_sensor_config *sensor_config, size_t channel_data_size,
                       const struct zmk_sensor_channel_data *channel_data) {
    ARG_UNUSED(sensor_config);
    ARG_UNUSED(channel_data_size);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_sensor_rotate_accel_data *data = dev->data;

    const int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);

    const struct sensor_value v = channel_data[0].value;
    int delta = (v.val1 == 0) ? v.val2 : v.val1;

    if (delta > 0) {
        data->pending_dir[sensor_index][event.layer] = ACCEL_DIR_CW;
    } else if (delta < 0) {
        data->pending_dir[sensor_index][event.layer] = ACCEL_DIR_CCW;
    } else {
        data->pending_dir[sensor_index][event.layer] = ACCEL_DIR_NONE;
    }

    return 0;
}

static int process(struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event,
                   enum behavior_sensor_binding_process_mode mode) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_sensor_rotate_accel_config *cfg = dev->config;
    struct behavior_sensor_rotate_accel_data *data = dev->data;

    const int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);

    if (mode != BEHAVIOR_SENSOR_BINDING_PROCESS_MODE_TRIGGER) {
        data->pending_dir[sensor_index][event.layer] = ACCEL_DIR_NONE;
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    enum accel_dir dir = data->pending_dir[sensor_index][event.layer];
    data->pending_dir[sensor_index][event.layer] = ACCEL_DIR_NONE;

    if (dir == ACCEL_DIR_NONE) {
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    // Central 側で確実に動かす
    event.source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL;
#endif

    uint32_t now = (uint32_t)k_uptime_get();
    uint32_t last = data->last_ms[sensor_index][event.layer];
    enum accel_dir last_dir = data->last_dir[sensor_index][event.layer];

    uint32_t dt = (last == 0) ? (uint32_t)cfg->dt_max_ms : (now - last);

    if (cfg->same_dir_only && last != 0 && last_dir != ACCEL_DIR_NONE && last_dir != dir) {
        // 逆回転で加速リセット（遅い扱い）
        dt = cfg->dt_max_ms;
    }

    data->last_ms[sensor_index][event.layer] = now;
    data->last_dir[sensor_index][event.layer] = dir;

    uint8_t steps = calc_steps(cfg, dt);

    struct zmk_behavior_binding base =
        (dir == ACCEL_DIR_CW) ? cfg->cw_binding : cfg->ccw_binding;

    struct zmk_behavior_binding scaled = scale_move_binding(base, steps);

    LOG_DBG("accel rotate pos=%d layer=%d dir=%d dt=%u steps=%u",
            event.position, event.layer, dir, dt, steps);

    enqueue_press(&event, scaled, cfg->tap_ms);
    enqueue_release(&event, scaled);

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api api = {
    .sensor_binding_accept_data = accept_data,
    .sensor_binding_process = process,
};

#define _TRANSFORM_ENTRY(idx, node)                                                                \
    {                                                                                              \
        .behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(node, bindings, idx)),               \
        .param1 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(node, bindings, idx, param1), (0),       \
                              (DT_INST_PHA_BY_IDX(node, bindings, idx, param1))),                  \
        .param2 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(node, bindings, idx, param2), (0),       \
                              (DT_INST_PHA_BY_IDX(node, bindings, idx, param2))),                  \
    }

#define INST(n)                                                                                    \
    static const struct behavior_sensor_rotate_accel_config cfg_##n = {                             \
        .cw_binding      = _TRANSFORM_ENTRY(0, n),                                                 \
        .ccw_binding     = _TRANSFORM_ENTRY(1, n),                                                 \
        .dt_min_ms       = DT_INST_PROP_OR(n, dt_min_ms, 30),                                      \
        .dt_max_ms       = DT_INST_PROP_OR(n, dt_max_ms, 200),                                     \
        .steps_min       = DT_INST_PROP_OR(n, steps_min, 1),                                       \
        .steps_max       = DT_INST_PROP_OR(n, steps_max, 5),                                       \
        .curve           = DT_INST_PROP_OR(n, curve, 1),                                           \
        .same_dir_only   = DT_INST_PROP_OR(n, same_dir_only, 0),                                   \
        .tap_ms          = DT_INST_PROP_OR(n, tap_ms, 0),                                          \
    };                                                                                             \
    static struct behavior_sensor_rotate_accel_data data_##n = {};                                 \
    BEHAVIOR_DT_INST_DEFINE(                                                                       \
        n, NULL, NULL, &data_##n, &cfg_##n,                                                        \
        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                          \
        &api);

DT_INST_FOREACH_STATUS_OKAY(INST)
