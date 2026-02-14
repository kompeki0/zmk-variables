/*
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT zmk_behavior_value_switch

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <zmk/behavior_queue.h>

#include "zmk_value_store.h"

#ifndef ZMK_KEYMAP_LEN
#define ZMK_KEYMAP_LEN 1
#endif

#ifndef ZMK_KEYMAP_LAYERS_LEN
#define ZMK_KEYMAP_LAYERS_LEN 1
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct behavior_value_switch_case {
    int32_t case_value;
    struct zmk_behavior_binding binding;
};

struct behavior_value_switch_config {
    uint8_t index;
    int32_t fallback;

    uint16_t case_count;
    const struct behavior_value_switch_case *cases;

    bool has_default_binding;
    struct zmk_behavior_binding default_binding;
};

struct behavior_value_switch_data {
    bool pressed_valid[ZMK_KEYMAP_LEN][ZMK_KEYMAP_LAYERS_LEN];
    struct zmk_behavior_binding pressed_binding[ZMK_KEYMAP_LEN][ZMK_KEYMAP_LAYERS_LEN];
};

static bool binding_is_valid(struct zmk_behavior_binding binding) {
    return binding.behavior_dev && binding.behavior_dev[0] != '\0';
}

static struct zmk_behavior_binding resolve_binding(const struct behavior_value_switch_config *cfg,
                                                   int32_t value) {
    for (int i = 0; i < cfg->case_count; i++) {
        if (cfg->cases[i].case_value == value) {
            return cfg->cases[i].binding;
        }
    }

    if (cfg->has_default_binding) {
        return cfg->default_binding;
    }

    struct zmk_behavior_binding none = {0};
    return none;
}

static bool state_index_valid(struct zmk_behavior_binding_event event) {
    return event.position >= 0 && event.position < ZMK_KEYMAP_LEN &&
           event.layer >= 0 && event.layer < ZMK_KEYMAP_LAYERS_LEN;
}

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_value_switch_config *cfg = dev->config;
    struct behavior_value_switch_data *data = dev->data;

    int32_t value = zmk_value_store_get(cfg->index, cfg->fallback);
    struct zmk_behavior_binding target = resolve_binding(cfg, value);

    if (!binding_is_valid(target)) {
        LOG_DBG("no case for value[%d]=%d", cfg->index, (int)value);
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    if (state_index_valid(event)) {
        data->pressed_binding[event.position][event.layer] = target;
        data->pressed_valid[event.position][event.layer] = true;
    }

    return zmk_behavior_queue_add(&event, target, true, 0);
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_value_switch_config *cfg = dev->config;
    struct behavior_value_switch_data *data = dev->data;

    struct zmk_behavior_binding target = {0};
    bool valid = false;

    if (state_index_valid(event) && data->pressed_valid[event.position][event.layer]) {
        target = data->pressed_binding[event.position][event.layer];
        data->pressed_valid[event.position][event.layer] = false;
        valid = true;
    } else {
        int32_t value = zmk_value_store_get(cfg->index, cfg->fallback);
        target = resolve_binding(cfg, value);
        valid = binding_is_valid(target);
    }

    if (!valid || !binding_is_valid(target)) {
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    return zmk_behavior_queue_add(&event, target, false, 0);
}

static const struct behavior_driver_api api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
};

#define _BINDING_ENTRY(idx, inst)                                                                    \
    {                                                                                                 \
        .behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(inst, bindings, idx)),                \
        .param1 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(inst, bindings, idx, param1), (0),        \
                              (DT_INST_PHA_BY_IDX(inst, bindings, idx, param1))),                    \
        .param2 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(inst, bindings, idx, param2), (0),        \
                              (DT_INST_PHA_BY_IDX(inst, bindings, idx, param2))),                    \
    }

#define _CASE_ENTRY(idx, inst)                                                                        \
    {                                                                                                 \
        .case_value = DT_INST_PROP_BY_IDX(inst, case_values, idx),                                   \
        .binding = _BINDING_ENTRY(idx, inst),                                                         \
    }

#define _DEFAULT_BINDING(inst)                                                                        \
    {                                                                                                 \
        .behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(inst, default_binding, 0)),            \
        .param1 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(inst, default_binding, 0, param1), (0),    \
                              (DT_INST_PHA_BY_IDX(inst, default_binding, 0, param1))),               \
        .param2 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(inst, default_binding, 0, param2), (0),    \
                              (DT_INST_PHA_BY_IDX(inst, default_binding, 0, param2))),               \
    }

#define INST(n)                                                                                        \
    BUILD_ASSERT(DT_INST_PROP_LEN(n, case_values) == DT_INST_PROP_LEN(n, bindings),                  \
                 "case-values and bindings must have the same length");                               \
    static const struct behavior_value_switch_case cases_##n[] = {                                    \
        LISTIFY(DT_INST_PROP_LEN(n, case_values), _CASE_ENTRY, (,), n)};                             \
    static struct behavior_value_switch_data data_##n = {0};                                          \
    static const struct behavior_value_switch_config cfg_##n = {                                      \
        .index = (uint8_t)DT_INST_PROP(n, index),                                                     \
        .fallback = DT_INST_PROP_OR(n, fallback, 0),                                                  \
        .case_count = DT_INST_PROP_LEN(n, case_values),                                               \
        .cases = cases_##n,                                                                           \
        .has_default_binding = DT_INST_NODE_HAS_PROP(n, default_binding),                            \
        .default_binding = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, default_binding),                    \
                                       (_DEFAULT_BINDING(n)),                                         \
                                       ({0})),                                                        \
    };                                                                                               \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, &data_##n, &cfg_##n,                                       \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api);

DT_INST_FOREACH_STATUS_OKAY(INST)
