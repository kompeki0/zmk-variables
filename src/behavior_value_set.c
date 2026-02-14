/*
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT zmk_behavior_value_set

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>

#include "zmk_value_store.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct behavior_value_set_config {
    uint8_t index;
    int32_t value;
};

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_value_set_config *cfg = dev->config;

    if (!zmk_value_store_set(cfg->index, cfg->value)) {
        LOG_WRN("failed to set value[%d] = %d", cfg->index, (int)cfg->value);
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    LOG_DBG("value[%d] = %d", cfg->index, (int)cfg->value);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
};

#define INST(n)                                                                                      \
    static const struct behavior_value_set_config cfg_##n = {                                        \
        .index = (uint8_t)DT_INST_PROP(n, index),                                                    \
        .value = DT_INST_PROP(n, value),                                                             \
    };                                                                                               \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, &cfg_##n,                                           \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api);

DT_INST_FOREACH_STATUS_OKAY(INST)
