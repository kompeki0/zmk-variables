#define DT_DRV_COMPAT zmk_behavior_mouse_move_dynamic

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/behavior_queue.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct cfg {
    const char *mmv_dev;
};

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct cfg *cfg = dev->config;

    struct zmk_behavior_binding inner = {
        .behavior_dev = cfg->mmv_dev,   // ← &mmv の device name
        .param1 = binding->param1,      // MOVE_UP 等
        .param2 = 0,
    };

    LOG_DBG("mmv_dyn press param1=%d inner=%s", binding->param1, inner.behavior_dev);

    zmk_behavior_queue_add(&event, inner, true, 0);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct cfg *cfg = dev->config;

    struct zmk_behavior_binding inner = {
        .behavior_dev = cfg->mmv_dev,
        .param1 = binding->param1,
        .param2 = 0,
    };

    zmk_behavior_queue_add(&event, inner, false, 0);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
};

#define INST(n) \
    static const struct cfg cfg_##n = { \
        .mmv_dev = DEVICE_DT_NAME(DT_INST_PHANDLE(n, binding)), \
    }; \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, &cfg_##n, \
        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api);

DT_INST_FOREACH_STATUS_OKAY(INST)
