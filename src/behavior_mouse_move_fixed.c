/*
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT zmk_behavior_mouse_move_fixed

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <dt-bindings/zmk/pointing.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * param1: MOVE_* encoded (x in low16, y in high16)
 * - press: start periodic input_report_rel
 * - release: stop
 */

struct behavior_mouse_move_fixed_config {
    int32_t speed;              // units/sec
    uint16_t trigger_period_ms; // ms
};

struct behavior_mouse_move_fixed_data {
    const struct device *dev;
    struct k_work_delayable tick_work;

    bool active;
    uint32_t dir_param; // MOVE_* encoded
};

static void tick_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct behavior_mouse_move_fixed_data *data =
        CONTAINER_OF(dwork, struct behavior_mouse_move_fixed_data, tick_work);

    if (!data->active || data->dev == NULL) {
        return;
    }

    const struct behavior_mouse_move_fixed_config *cfg = data->dev->config;

    uint16_t period_ms = cfg->trigger_period_ms ? cfg->trigger_period_ms : 20;
    int32_t speed = cfg->speed;
    if (speed < 1) speed = 1;

    // periodあたりの移動量
    int32_t step32 = (speed * (int32_t)period_ms) / 1000;
    if (step32 < 1) step32 = 1;
    if (step32 > INT16_MAX) step32 = INT16_MAX;
    int16_t step = (int16_t)step32;

    int16_t xdir = MOVE_X_DECODE(data->dir_param);
    int16_t ydir = MOVE_Y_DECODE(data->dir_param);

    int16_t dx = 0, dy = 0;
    if (xdir != 0) dx = (xdir > 0) ? step : (int16_t)-step;
    if (ydir != 0) dy = (ydir > 0) ? step : (int16_t)-step;

    LOG_DBG("mmv_fixed tick dir=(%d,%d) step=%d dx=%d dy=%d", xdir, ydir, step, dx, dy);

    if (dx != 0 && dy != 0) {
        input_report_rel(data->dev, INPUT_REL_X, dx, false, K_NO_WAIT);
        input_report_rel(data->dev, INPUT_REL_Y, dy, true,  K_NO_WAIT);
    } else if (dx != 0) {
        input_report_rel(data->dev, INPUT_REL_X, dx, true, K_NO_WAIT);
    } else if (dy != 0) {
        input_report_rel(data->dev, INPUT_REL_Y, dy, true, K_NO_WAIT);
    }

    k_work_schedule(&data->tick_work, K_MSEC(period_ms));
}

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_mouse_move_fixed_data *data = dev->data;

    data->dev = dev;
    data->active = true;
    data->dir_param = binding->param1;

    LOG_DBG("mmv_fixed press dir_param=0x%08x", data->dir_param);

    k_work_schedule(&data->tick_work, K_NO_WAIT);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_mouse_move_fixed_data *data = dev->data;

    LOG_DBG("mmv_fixed release");
    data->active = false;
    k_work_cancel_delayable(&data->tick_work);

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
};

static int init(const struct device *dev) {
    struct behavior_mouse_move_fixed_data *data = dev->data;
    data->dev = dev;
    k_work_init_delayable(&data->tick_work, tick_work_cb);
    return 0;
}

#define INST(n)                                                                                 \
    static struct behavior_mouse_move_fixed_data data_##n = {};                                 \
    static const struct behavior_mouse_move_fixed_config cfg_##n = {                            \
        .speed = DT_INST_PROP_OR(n, speed, 1200),                                               \
        .trigger_period_ms = DT_INST_PROP_OR(n, trigger_period_ms, 20),                         \
    };                                                                                          \
    BEHAVIOR_DT_INST_DEFINE(n, init, NULL, &data_##n, &cfg_##n,                                  \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api);

DT_INST_FOREACH_STATUS_OKAY(INST)
