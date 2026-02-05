/*
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT zmk_behavior_mouse_move_value

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <dt-bindings/zmk/pointing.h>

#include <zmk/zmk_value_store.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct behavior_mouse_move_value_config {
    uint8_t index;

    int32_t value_min;
    int32_t value_max;

    int32_t min_speed;
    int32_t max_speed;

    uint16_t trigger_period_ms;
};

struct behavior_mouse_move_value_data {
    const struct device *dev;
    struct k_work_delayable tick_work;

    bool active;
    uint32_t dir_param; // MOVE_* encoded (x in low16, y in high16)
};

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static int32_t map_value_to_speed(const struct behavior_mouse_move_value_config *cfg, int32_t v) {
    int32_t vmin = cfg->value_min;
    int32_t vmax = cfg->value_max;
    if (vmax == vmin) return cfg->max_speed;

    v = clamp_i32(v, vmin, vmax);

    int64_t num = (int64_t)(v - vmin) * (cfg->max_speed - cfg->min_speed);
    int64_t den = (int64_t)(vmax - vmin);
    int32_t out = cfg->min_speed + (int32_t)(num / den);

    return clamp_i32(out, cfg->min_speed, cfg->max_speed);
}

static void tick_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct behavior_mouse_move_value_data *data =
        CONTAINER_OF(dwork, struct behavior_mouse_move_value_data, tick_work);

    if (!data->active || data->dev == NULL) return;

    const struct behavior_mouse_move_value_config *cfg = data->dev->config;

    int32_t raw = zmk_value_store_get(cfg->index, cfg->value_min);
    int32_t speed = map_value_to_speed(cfg, raw);

    int16_t xdir = (int16_t)(data->dir_param & 0xFFFF);
    int16_t ydir = (int16_t)((data->dir_param >> 16) & 0xFFFF);

    // periodあたりの移動量: speed[unit/sec] * period[ms] / 1000
    int32_t step = (speed * cfg->trigger_period_ms) / 1000;
    if (step < 1) step = 1;

    int16_t dx = 0, dy = 0;
    if (xdir != 0) dx = (xdir > 0) ? (int16_t)step : (int16_t)(-step);
    if (ydir != 0) dy = (ydir > 0) ? (int16_t)step : (int16_t)(-step);

    // 送信（x/y 両方ある場合は最後の引数 sync を調整）
    if (dx != 0 && dy != 0) {
        input_report_rel(data->dev, INPUT_REL_X, dx, false, K_NO_WAIT);
        input_report_rel(data->dev, INPUT_REL_Y, dy, true, K_NO_WAIT);
    } else if (dx != 0) {
        input_report_rel(data->dev, INPUT_REL_X, dx, true, K_NO_WAIT);
    } else if (dy != 0) {
        input_report_rel(data->dev, INPUT_REL_Y, dy, true, K_NO_WAIT);
    }

    k_work_schedule(&data->tick_work, K_MSEC(cfg->trigger_period_ms));
}

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_mouse_move_value_data *data = dev->data;

    data->dev = dev;
    data->active = true;
    data->dir_param = binding->param1; // MOVE_* encoded

    // 即開始
    k_work_schedule(&data->tick_work, K_NO_WAIT);

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_mouse_move_value_data *data = dev->data;

    data->active = false;
    k_work_cancel_delayable(&data->tick_work);

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
};

static int init(const struct device *dev) {
    struct behavior_mouse_move_value_data *data = dev->data;
    data->dev = dev;
    k_work_init_delayable(&data->tick_work, tick_work_cb);
    return 0;
}

#define INST(n)                                                                                \
    static struct behavior_mouse_move_value_data data_##n = {};                                \
    static const struct behavior_mouse_move_value_config cfg_##n = {                           \
        .index = DT_INST_PROP_OR(n, index, 0),                                                 \
        .value_min = DT_INST_PROP_OR(n, value_min, 0),                                         \
        .value_max = DT_INST_PROP_OR(n, value_max, 100),                                       \
        .min_speed = DT_INST_PROP_OR(n, min_speed, 200),                                       \
        .max_speed = DT_INST_PROP_OR(n, max_speed, 1800),                                      \
        .trigger_period_ms = DT_INST_PROP_OR(n, trigger_period_ms, 20),                        \
    };                                                                                         \
    BEHAVIOR_DT_INST_DEFINE(n, init, NULL, &data_##n, &cfg_##n,                                 \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api);

DT_INST_FOREACH_STATUS_OKAY(INST)
