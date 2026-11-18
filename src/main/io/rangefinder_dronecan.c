/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#if defined(USE_RANGEFINDER_DRONECAN)

#include <dronecan_msgs.h>

#include "drivers/rangefinder/rangefinder_virtual.h"

#include "io/rangefinder.h"

static bool hasNewData = false;
static int32_t sensorData = RANGEFINDER_NO_NEW_DATA;

static bool dronecanRangefinderDetect(void)
{
    return true;
}

static void dronecanRangefinderInit(void)
{
}

static void dronecanRangefinderUpdate(void)
{
}

static int32_t dronecanRangefinderGetDistance(void)
{
    if (hasNewData) {
        hasNewData = false;
        return (sensorData > 0) ? sensorData : RANGEFINDER_OUT_OF_RANGE;
    }

    return RANGEFINDER_NO_NEW_DATA;
}

// Called for every uavcan.equipment.range_sensor.Measurement on the bus.
// Anything that is not a valid range still counts as a reply from the sensor,
// so it is reported as out of range rather than dropped.
void dronecanRangefinderReceive(float rangeMeters, uint8_t readingType)
{
    const bool isValid = readingType == UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE &&
        isfinite(rangeMeters);

    sensorData = isValid ? lrintf(rangeMeters * 100.0f) : RANGEFINDER_OUT_OF_RANGE;
    hasNewData = true;
}

virtualRangefinderVTable_t rangefinderDronecanVtable = {
    .detect = dronecanRangefinderDetect,
    .init = dronecanRangefinderInit,
    .update = dronecanRangefinderUpdate,
    .read = dronecanRangefinderGetDistance
};

#endif
