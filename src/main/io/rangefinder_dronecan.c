/*
 * This file is part of INAV Project.
 *
 * DroneCAN rangefinder (uavcan.equipment.range_sensor.Measurement, id 1050).
 * ARK Flow publishes its lidar on this message, on the same bus as its
 * optical flow. This is not the ARK DIST sensor; that one is UART MAVLink.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"

#if defined(USE_RANGEFINDER_DRONECAN)

#include "build/atomic.h"

#include "drivers/nvic.h"
#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_virtual.h"

#include "io/rangefinder.h"

#include "uavcan.equipment.range_sensor.Measurement.h"

static bool hasNewData = false;
static int32_t sensorData = RANGEFINDER_NO_NEW_DATA;

static bool dronecanRangefinderDetect(void)
{
    return true;
}

static void dronecanRangefinderInit(void)
{
    hasNewData = false;
    sensorData = RANGEFINDER_NO_NEW_DATA;
}

static void dronecanRangefinderUpdate(void)
{
}

static int32_t dronecanRangefinderGetDistance(void)
{
    bool newData;
    int32_t sample;

    ATOMIC_BLOCK(NVIC_PRIO_CAN) {
        newData = hasNewData;
        sample = sensorData;
        hasNewData = false;
    }

    if (newData) {
        return sample;
    }

    return RANGEFINDER_NO_NEW_DATA;
}

void dronecanRangefinderReceiveMeasurement(const struct uavcan_equipment_range_sensor_Measurement *msg)
{
    if (!msg) {
        return;
    }

    int32_t sample = RANGEFINDER_NO_NEW_DATA;
    bool publish = false;

    switch (msg->reading_type) {
        case UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE:
            if (!isfinite(msg->range) || msg->range <= 0.0f) {
                sample = RANGEFINDER_OUT_OF_RANGE;
            } else {
                sample = (int32_t)(msg->range * 100.0f + 0.5f);
            }
            publish = true;
            break;

        case UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_CLOSE:
        case UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR:
            sample = RANGEFINDER_OUT_OF_RANGE;
            publish = true;
            break;

        default:
            break;
    }

    if (!publish) {
        return;
    }

    ATOMIC_BLOCK(NVIC_PRIO_CAN) {
        sensorData = sample;
        hasNewData = true;
    }
}

virtualRangefinderVTable_t rangefinderDronecanVtable = {
    .detect = dronecanRangefinderDetect,
    .init = dronecanRangefinderInit,
    .update = dronecanRangefinderUpdate,
    .read = dronecanRangefinderGetDistance
};

#endif
