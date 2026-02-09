/*
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT zmk_behavior_input_one_shot_two_axis

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h> // CLAMP

#include <zmk/behavior.h>
#include <dt-bindings/zmk/pointing.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct behavior_input_one_shot_two_axis_config {
    int16_t x_code;      // e.g. INPUT_REL_HWHEEL
    int16_t y_code;      // e.g. INPUT_REL_WHEEL
    int16_t scale;       // 1ノッチあたりの出力倍率（デフォ1）
};

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_input_one_shot_two_axis_config *cfg = dev->config;

    int16_t x = MOVE_X_DECODE(binding->param1);
    int16_t y = MOVE_Y_DECODE(binding->param1);

    // 1回だけ送る（tap用途）
    int16_t out_x = (int16_t)CLAMP((int32_t)x * cfg->scale, INT16_MIN, INT16_MAX);
    int16_t out_y = (int16_t)CLAMP((int32_t)y * cfg->scale, INT16_MIN, INT16_MAX);

    LOG_DBG("one-shot pos=%d param=0x%02X x=%d y=%d out_x=%d out_y=%d",
            event.position, binding->param1, x, y, out_x, out_y);

    int ret = 0;
    bool have_x = out_x != 0;
    bool have_y = out_y != 0;

    // input_report_rel の “sync” 引数は最後の報告だけ true にするのが流儀
    if (have_x) {
        ret = input_report_rel(dev, cfg->x_code, out_x, !have_y, K_NO_WAIT);
    }
    if (have_y) {
        ret = input_report_rel(dev, cfg->y_code, out_y, true, K_NO_WAIT);
    }

    return ret;
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    // one-shotなので何もしない
    return 0;
}

static const struct behavior_driver_api api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
};

#define INST(n)                                                                                    \
    static const struct behavior_input_one_shot_two_axis_config cfg_##n = {                         \
        .x_code = DT_INST_PROP(n, x_input_code),                                                    \
        .y_code = DT_INST_PROP(n, y_input_code),                                                    \
        .scale  = DT_INST_PROP_OR(n, scale, 1),                                                     \
    };                                                                                              \
    BEHAVIOR_DT_INST_DEFINE(                                                                        \
        n, NULL, NULL, NULL, &cfg_##n,                                                              \
        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api);

DT_INST_FOREACH_STATUS_OKAY(INST)
