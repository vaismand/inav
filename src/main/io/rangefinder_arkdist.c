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

/*
 * ARK DIST SR over UART MAVLink.
 * The sensor publishes DISTANCE_SENSOR at 115200 8N1, sysid/compid 158,
 * and switches between its short-range and long-range rates on its own.
 * https://docs.arkelectron.com/products/sensor/ark-dist/uart-mavlink
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/build_config.h"

#include "io/serial.h"

#if defined(USE_RANGEFINDER_ARKDIST)
#include "drivers/rangefinder/rangefinder_virtual.h"
#include "drivers/time.h"
#include "io/rangefinder.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#define MAVLINK_COMM_NUM_BUFFERS 1
#include "common/mavlink.h"
#pragma GCC diagnostic pop

#define ARKDIST_MAVLINK_BAUD               115200
#define ARKDIST_MAVLINK_SYSID              158
#define ARKDIST_MAVLINK_COMPID             158

static serialPort_t *serialPort = NULL;
static serialPortConfig_t *portConfig;
static mavlink_message_t mavMsg;
static mavlink_status_t mavStatus;

static bool hasNewData = false;
static int32_t sensorData = RANGEFINDER_NO_NEW_DATA;

static bool arkdistMessageSourceIsValid(const mavlink_message_t *msg)
{
    return (msg->sysid == ARKDIST_MAVLINK_SYSID) && (msg->compid == ARKDIST_MAVLINK_COMPID);
}

static bool arkdistDistanceReadingIsValid(const mavlink_distance_sensor_t *msg, int32_t distanceCm)
{
    if (msg->signal_quality == 1) {
        // MAVLink: 1 means the signal is invalid. 0 means quality was not set.
        return false;
    }

    if (distanceCm == 0) {
        return false;
    }

    if (msg->min_distance > 0 && distanceCm < msg->min_distance) {
        return false;
    }

    if (msg->max_distance > 0 && distanceCm > msg->max_distance) {
        return false;
    }

    if (distanceCm < ARKDIST_MIN_RANGE_CM || distanceCm > ARKDIST_MAX_RANGE_CM) {
        return false;
    }

    return true;
}

static bool arkdistRangefinderDetect(void)
{
    portConfig = findSerialPortConfig(FUNCTION_RANGEFINDER);
    if (!portConfig) {
        return false;
    }

    return true;
}

static void arkdistRangefinderInit(void)
{
    if (!portConfig) {
        return;
    }

    // RX/TX/GND/5V only. No RTS/CTS is wired, so open 8N1 with no flow control.
    serialPort = openSerialPort(portConfig->identifier, FUNCTION_RANGEFINDER, NULL, NULL, ARKDIST_MAVLINK_BAUD, MODE_RX, SERIAL_NOT_INVERTED);
    if (!serialPort) {
        return;
    }

    memset(&mavStatus, 0, sizeof(mavStatus));
    hasNewData = false;
    sensorData = RANGEFINDER_NO_NEW_DATA;
}

static void arkdistRangefinderUpdate(void)
{
    if (!serialPort) {
        return;
    }

    while (serialRxBytesWaiting(serialPort) > 0) {
        const uint8_t c = serialRead(serialPort);

        if (mavlink_parse_char(MAVLINK_COMM_0, c, &mavMsg, &mavStatus) != MAVLINK_FRAMING_OK) {
            continue;
        }

        if (mavMsg.msgid != MAVLINK_MSG_ID_DISTANCE_SENSOR) {
            continue;
        }

        if (!arkdistMessageSourceIsValid(&mavMsg)) {
            continue;
        }

        mavlink_distance_sensor_t msg;
        mavlink_msg_distance_sensor_decode(&mavMsg, &msg);

        const int32_t distanceCm = msg.current_distance;

        hasNewData = true;
        sensorData = arkdistDistanceReadingIsValid(&msg, distanceCm) ? distanceCm : RANGEFINDER_OUT_OF_RANGE;
    }
}

static int32_t arkdistRangefinderGetDistance(void)
{
    if (hasNewData) {
        hasNewData = false;
        return (sensorData > 0) ? sensorData : RANGEFINDER_OUT_OF_RANGE;
    }
    else {
        return RANGEFINDER_NO_NEW_DATA;
    }
}

virtualRangefinderVTable_t rangefinderArkdistVtable = {
    .detect = arkdistRangefinderDetect,
    .init = arkdistRangefinderInit,
    .update = arkdistRangefinderUpdate,
    .read = arkdistRangefinderGetDistance
};

#endif
