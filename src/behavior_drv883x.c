/*
 * Copyright (c) 2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_drv883x

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>

#include <zmk/behavior.h>
#include <zmk/hid.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <stdlib.h>

#include <zmk/drivers/drv883x.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// #if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct behavior_drv883x_config {
    const struct device *drv883x_dev;
    uint32_t vel_neutral;
    uint32_t vel_min_max;
};

struct behavior_drv883x_data {
    const struct device *dev;
};

static int drv883x_binding_pressed(struct zmk_behavior_binding *binding,
                                   struct zmk_behavior_binding_event event) {
    LOG_DBG("position %d drv883x: 0x%02X 0x%02X", event.position, binding->param1, binding->param2);
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    // struct behavior_drv883x_data *data = (struct behavior_drv883x_data *)dev->data;
    const struct behavior_drv883x_config *cfg = dev->config;

    int err = 0;
    const struct device *drv883x_dev = cfg->drv883x_dev;
    struct sensor_value val = { .val1 = 0, .val2 = 0 };

    uint32_t chan = binding->param1;
    int16_t vel = binding->param2 - cfg->vel_neutral;

    // crop min-max velocity value
    if (vel > 0 && vel > cfg->vel_min_max) {
        vel = cfg->vel_min_max;
    } else if (vel < 0 && vel < -cfg->vel_min_max) {
        vel = -cfg->vel_min_max;
    }

    // enable enable pin, disable iif vel == 0
    val.val1 = (!vel) ? 0 : 1;
    val.val2 = 0;
    err = sensor_attr_set(drv883x_dev, SENSOR_CHAN_ALL, 
                          (enum sensor_attribute) DRV883X_ATTR_ENABLE, &val);
    if (err) {
        LOG_WRN("Fail to sensor_attr_set DRV883X_ATTR_ENABLE");
        return -EIO;
    }

    // set velocity and inversr flag
    val.val1 = abs(vel);
    val.val2 = vel < 0;
    err = sensor_attr_set(drv883x_dev, SENSOR_CHAN_ALL, 
                          (enum sensor_attribute) DRV883X_ATTR_VELOCITY, &val);
    if (err) {
        LOG_WRN("Fail to sensor_attr_set DRV883X_ATTR_VELOCITY");
        return -EIO;
    }

    // call sync to latch velocity setting to driver
    //
    // ** NOTE**: val1 is ignored currently. only ONE channel is implemented.
    //
    val.val1 = chan;
    val.val2 = 0;
    err = sensor_attr_set(drv883x_dev, SENSOR_CHAN_ALL, 
                          (enum sensor_attribute) DRV883X_ATTR_SYNC, &val);
    if (err) {
        LOG_WRN("Fail to sensor_attr_set DRV883X_ATTR_SYNC");
        return -EIO;
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static int drv883x_binding_released(struct zmk_behavior_binding *binding,
                                    struct zmk_behavior_binding_event event) {
    LOG_DBG("position %d drv883x: 0x%02X 0x%02X", event.position, binding->param1, binding->param2);
    // const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int behavior_drv883x_init(const struct device *dev) {
    struct behavior_drv883x_data *data = dev->data;
    data->dev = dev;
    return 0;
};

static const struct behavior_driver_api behavior_drv883x_driver_api = {
    .binding_pressed = drv883x_binding_pressed,
    .binding_released = drv883x_binding_released,
};

#define ZMK_BEHAVIOR_DRV883X_PRIORITY 11

#define DRV883X_BEH_INST(n)                                             \
    static struct behavior_drv883x_data data_##n = {};                  \
    static struct behavior_drv883x_config config_##n = {                \
        .drv883x_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, drv883x_dev)),  \
        .vel_neutral = DT_PROP(DT_DRV_INST(n), vel_neutral),            \
        .vel_min_max = DT_PROP(DT_DRV_INST(n), vel_min_max),            \
    };                                                                  \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_drv883x_init, NULL,             \
                            &data_##n, &config_##n,                     \
                            POST_KERNEL,                                \
                            ZMK_BEHAVIOR_DRV883X_PRIORITY,              \
                            &behavior_drv883x_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DRV883X_BEH_INST)

// #endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
