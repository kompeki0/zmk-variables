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

// mmv (&mmv) の公開関数
int behavior_input_two_axis_adjust_speed(const struct device *dev, int16_t dx, int16_t dy);

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct behavior_mouse_move_value_config {
    const char *target_behavior; // binding = <&mmv> -> DEVICE_DT_NAME()
    uint8_t index;

    int32_t value_min;
    int32_t value_max;

    int32_t min_speed;
    int32_t max_speed;

    uint16_t trigger_period_ms;

    // 斜め正規化（ベクトル長を一定にする）
    bool normalize_diagonal;
    // 1/sqrt(2) ≒ 0.707 を permil で近似
    uint16_t diag_scale_permil; // default 707
};

struct behavior_mouse_move_value_data {
    const struct device *self_dev;

    // 押下方向カウント（同時押し対応）
    int8_t x_pos; // RIGHT
    int8_t x_neg; // LEFT
    int8_t y_pos; // DOWN
    int8_t y_neg; // UP

    // mmvへ現在適用している合成速度
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

static inline int8_t sign_from_counts(int8_t pos, int8_t neg) {
    int16_t d = (int16_t)pos - (int16_t)neg;
    if (d > 0) return 1;
    if (d < 0) return -1;
    return 0;
}

static bool have_any_press(const struct behavior_mouse_move_value_data *data) {
    return (data->x_pos || data->x_neg || data->y_pos || data->y_neg);
}

static void recompute_and_apply(const struct device *self_dev) {
    const struct behavior_mouse_move_value_config *cfg = self_dev->config;
    struct behavior_mouse_move_value_data *data = self_dev->data;

    const struct device *mmv_dev = device_get_binding(cfg->target_behavior);
    if (!mmv_dev) return;

    if (!have_any_press(data)) {
        // 何も押されていないなら、適用中の分を戻してゼロに
        if (data->cur_dx != 0 || data->cur_dy != 0) {
            behavior_input_two_axis_adjust_speed(mmv_dev, (int16_t)-data->cur_dx, (int16_t)-data->cur_dy);
            data->cur_dx = 0;
            data->cur_dy = 0;
        }
        return;
    }

    int32_t raw = zmk_value_store_get(cfg->index, cfg->value_min);
    int32_t base = map_value_to_speed(cfg, raw);
    if (base < 1) base = 1;
    if (base > INT16_MAX) base = INT16_MAX;

    int8_t sx = sign_from_counts(data->x_pos, data->x_neg);
    int8_t sy = sign_from_counts(data->y_pos, data->y_neg);

    int32_t want_dx32 = (sx == 0) ? 0 : (sx > 0 ? base : -base);
    int32_t want_dy32 = (sy == 0) ? 0 : (sy > 0 ? base : -base);

    // 斜め正規化：x,y 両方動いているなら 1/sqrt(2) 近似で両方縮める
    if (cfg->normalize_diagonal && want_dx32 != 0 && want_dy32 != 0) {
        uint16_t s = cfg->diag_scale_permil ? cfg->diag_scale_permil : 707;
        want_dx32 = (want_dx32 * (int32_t)s) / 1000;
        want_dy32 = (want_dy32 * (int32_t)s) / 1000;
        if (want_dx32 == 0) want_dx32 = (sx > 0) ? 1 : -1;
        if (want_dy32 == 0) want_dy32 = (sy > 0) ? 1 : -1;
    }

    int16_t want_dx = (int16_t)clamp_i32(want_dx32, INT16_MIN, INT16_MAX);
    int16_t want_dy = (int16_t)clamp_i32(want_dy32, INT16_MIN, INT16_MAX);

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

    if (!data->self_dev) return;

    // 値が変わっていれば追随（押下中のみ意味がある）
    if (have_any_press(data)) {
        recompute_and_apply(data->self_dev);
        const struct behavior_mouse_move_value_config *cfg = data->self_dev->config;
        uint16_t ms = cfg->trigger_period_ms ? cfg->trigger_period_ms : 40;
        k_work_schedule(&data->tick_work, K_MSEC(ms));
    }
}

static void bump_counts_from_param(struct behavior_mouse_move_value_data *data, uint32_t param, int8_t delta) {
    int16_t x = MOVE_X_DECODE(param);
    int16_t y = MOVE_Y_DECODE(param);

    // MOVE_LEFT/RIGHT は x に符号が入る、MOVE_UP/DOWN は y に符号が入る
    if (x > 0) data->x_pos = (int8_t)(data->x_pos + delta);
    else if (x < 0) data->x_neg = (int8_t)(data->x_neg + delta);

    if (y > 0) data->y_pos = (int8_t)(data->y_pos + delta);
    else if (y < 0) data->y_neg = (int8_t)(data->y_neg + delta);

    // 安全策：負数になったらゼロへ戻す（押下/解放の不整合対策）
    if (data->x_pos < 0) data->x_pos = 0;
    if (data->x_neg < 0) data->x_neg = 0;
    if (data->y_pos < 0) data->y_pos = 0;
    if (data->y_neg < 0) data->y_neg = 0;
}

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_mouse_move_value_data *data = dev->data;

    bump_counts_from_param(data, binding->param1, +1);

    // 合成速度を再計算して適用
    recompute_and_apply(dev);

    // 押下がある限り、値の変化に追随
    const struct behavior_mouse_move_value_config *cfg = dev->config;
    uint16_t ms = cfg->trigger_period_ms ? cfg->trigger_period_ms : 40;
    k_work_schedule(&data->tick_work, K_MSEC(ms));

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_mouse_move_value_data *data = dev->data;

    bump_counts_from_param(data, binding->param1, -1);

    // 合成速度を再計算（0になったら自動で戻す）
    recompute_and_apply(dev);

    // 全解放なら tick を止める
    if (!have_any_press(data)) {
        k_work_cancel_delayable(&data->tick_work);
    }

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
        .normalize_diagonal = DT_INST_PROP_OR(n, normalize_diagonal, 1),                            \
        .diag_scale_permil = DT_INST_PROP_OR(n, diag_scale_permil, 707),                            \
    };                                                                                              \
    BEHAVIOR_DT_INST_DEFINE(n, init, NULL, &data_##n, &cfg_##n,                                      \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api);

DT_INST_FOREACH_STATUS_OKAY(INST)
