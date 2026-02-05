/*
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT zmk_behavior_mouse_move_value

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <dt-bindings/zmk/pointing.h>

#include <zmk_value_store.h>

int behavior_input_two_axis_adjust_speed(const struct device *dev, int16_t dx, int16_t dy);

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct behavior_mouse_move_value_config {
    const char *target_behavior; // DEVICE_DT_NAME(DT_INST_PHANDLE(...))
    uint8_t index;

    int32_t value_min;
    int32_t value_max;

    int32_t min_speed;
    int32_t max_speed;

    uint16_t trigger_period_ms;
};

struct behavior_mouse_move_value_data {
    const struct device *self_dev;

    bool active;
    uint32_t dir_param;

    int16_t cur_dx;
    int16_t cur_dy;

    struct k_work_delayable tick_work;
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

static void apply_speed(const struct device *self_dev) {
    const struct behavior_mouse_move_value_config *cfg = self_dev->config;
    struct behavior_mouse_move_value_data *data = self_dev->data;

    const struct device *mmv_dev = device_get_binding(cfg->target_behavior);
    if (mmv_dev == NULL) return;

    int32_t raw = zmk_value_store_get(cfg->index, cfg->value_min);
    int32_t speed = map_value_to_speed(cfg, raw);
    if (speed < 1) speed = 1;
    if (speed > INT16_MAX) speed = INT16_MAX;

    int16_t xdir = MOVE_X_DECODE(data->dir_param);
    int16_t ydir = MOVE_Y_DECODE(data->dir_param);

    int16_t want_dx = (xdir == 0) ? 0 : (xdir > 0 ? (int16_t)speed : (int16_t)-speed);
    int16_t want_dy = (ydir == 0) ? 0 : (ydir > 0 ? (int16_t)speed : (int16_t)-speed);

    int16_t delta_x = (int16_t)(want_dx - data->cur_dx);
    int16_t delta_y = (int16_t)(want_dy - data->cur_dy);

    if (delta_x != 0 || delta_y != 0) {
        behavior_input_two_axis_adjust_speed(mmv_dev, delta_x, delta_y);
        data->cur_dx = want_dx;
        data->cur_dy = want_dy;
    }
}

static void tick_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct behavior_mouse_move_value_data *data =
        CONTAINER_OF(dwork, struct behavior_mouse_move_value_data, tick_work);

    if (!data->active || data->self_dev == NULL) return;

    apply_speed(data->self_dev);

    const struct behavior_mouse_move_value_config *cfg = data->self_dev->config;
    uint16_t ms = cfg->trigger_period_ms ? cfg->trigger_period_ms : 40;
    k_work_schedule(&data->tick_work, K_MSEC(ms));
}

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_mouse_move_value_data *data = dev->data;

    data->active = true;
    data->dir_param = binding->param1;

    // 初回適用＋定期更新開始
    apply_speed(dev);

    const struct behavior_mouse_move_value_config *cfg = dev->config;
    uint16_t ms = cfg->trigger_period_ms ? cfg->trigger_period_ms : 40;
    k_work_schedule(&data->tick_work, K_MSEC(ms));

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_mouse_move_value_data *data = dev->data;

    if (!data->active) return ZMK_BEHAVIOR_OPAQUE;

    const struct behavior_mouse_move_value_config *cfg = dev->config;
    const struct device *mmv_dev = device_get_binding(cfg->target_behavior);

    k_work_cancel_delayable(&data->tick_work);

    // 適用していた分を戻す
    if (mmv_dev) {
        behavior_input_two_axis_adjust_speed(mmv_dev, (int16_t)-data->cur_dx, (int16_t)-data->cur_dy);
    }

    data->active = false;
    data->cur_dx = 0;
    data->cur_dy = 0;

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
};

static int init(const struct device *dev) {
    struct behavior_mouse_move_value_data *data = dev->data;
    data->self_dev = dev;
    k_work_init_delayable(&data->tick_work, tick_work_cb);
    return 0;
}

#define INST(n)                                                                                     \
    static struct behavior_mouse_move_value_data data_##n = {};                                     \
    static const struct behavior_mouse_move_value_config cfg_##n = {                                \
        .target_behavior = DEVICE_DT_NAME(DT_INST_PHANDLE(n, binding)),                             \
        .index = DT_INST_PROP_OR(n, index, 0),                                                      \
        .value_min = DT_INST_PROP_OR(n, value_min, 0),                                              \
        .value_max = DT_INST_PROP_OR(n, value_max, 100),                                            \
        .min_speed = DT_INST_PROP_OR(n, min_speed, 200),                                            \
        .max_speed = DT_INST_PROP_OR(n, max_speed, 1800),                                           \
        .trigger_period_ms = DT_INST_PROP_OR(n, trigger_period_ms, 40),                             \
    };                                                                                              \
    BEHAVIOR_DT_INST_DEFINE(n, init, NULL, &data_##n, &cfg_##n,                                      \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api);

DT_INST_FOREACH_STATUS_OKAY(INST)
