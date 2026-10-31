/*
 * This file is part of INAV Project.
 *
 * ARK Flow / HereFlow optical flow over DroneCAN
 * (com.hex.equipment.flow.Measurement, id 20200).
 *
 * The node publishes flow_integral in radians over integration_interval,
 * already in the sensor body frame. Messages that arrive between optical-flow
 * polls are summed and converted to degrees once. With opflow_scale = 1.0
 * that degree integral becomes deg/s in the existing optical-flow sensor
 * path. The global opflow_scale default (10.5) is for CXOF counts and is
 * left unchanged.
 *
 * The node's rate_gyro_integral is not used. Body-rate compensation stays
 * with the flight-controller gyro, the same path CXOF and MSP flow use.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"

#if defined(USE_OPFLOW_DRONECAN)

#include "build/atomic.h"

#include "common/maths.h"

#include "drivers/nvic.h"
#include "drivers/opflow/opflow_virtual.h"
#include "drivers/time.h"

#include "io/opflow.h"

#include "sensors/opflow.h"

#include "com.hex.equipment.flow.Measurement.h"

#define OPFLOW_DRONECAN_GAP_US              100000
#define OPFLOW_DRONECAN_MAX_INTERVAL_US     100000
#define OPFLOW_DRONECAN_DT_MIN_US           1000
#define OPFLOW_DRONECAN_DT_MAX_US           200000

typedef struct opflowCanAcc_s {
    float sumFlow[2];
    uint32_t sumIntervalUs;
    uint8_t minQuality;
    uint16_t count;
} opflowCanAcc_t;

static opflowCanAcc_t acc;
static timeUs_t prevArrivalUs;

static void opflowCanAccReset(opflowCanAcc_t *slot)
{
    memset(slot, 0, sizeof(*slot));
    slot->minQuality = 255;
}

static bool dronecanOpflowDetect(void)
{
    return true;
}

static bool dronecanOpflowInit(void)
{
    opflowCanAccReset(&acc);
    prevArrivalUs = 0;
    return true;
}

static bool dronecanOpflowUpdate(opflowData_t *data)
{
    opflowCanAcc_t snap;

    ATOMIC_BLOCK(NVIC_PRIO_CAN) {
        snap = acc;
        opflowCanAccReset(&acc);
    }

    if (snap.count == 0) {
        return false;
    }

    uint32_t dtUs = snap.sumIntervalUs;
    if (dtUs < OPFLOW_DRONECAN_DT_MIN_US) {
        dtUs = OPFLOW_DRONECAN_DT_MIN_US;
    } else if (dtUs > OPFLOW_DRONECAN_DT_MAX_US) {
        dtUs = OPFLOW_DRONECAN_DT_MAX_US;
    }

    data->deltaTime = (timeDelta_t)dtUs;
    data->flowRateRaw[0] = RADIANS_TO_DEGREES(snap.sumFlow[0]);
    data->flowRateRaw[1] = RADIANS_TO_DEGREES(snap.sumFlow[1]);
    data->flowRateRaw[2] = 0;
    /*
     * Quality is already 0..255. The MSP driver rescales 0..100, which would
     * put the valid threshold (35) above what this sensor reports in the dark.
     */
    data->quality = snap.minQuality;
    return true;
}

void dronecanOpflowReceiveMeasurement(const struct com_hex_equipment_flow_Measurement *msg)
{
    if (!msg) {
        return;
    }

    const bool intervalFinite = isfinite(msg->integration_interval) && msg->integration_interval >= 0.0001f;
    if (!intervalFinite || !isfinite(msg->flow_integral[0]) || !isfinite(msg->flow_integral[1])) {
        return;
    }

    const timeUs_t nowUs = micros();
    const uint32_t intervalUs = (uint32_t)lrintf(msg->integration_interval * 1.0e6f);
    const uint32_t arrivalGapUs = (prevArrivalUs != 0) ? (uint32_t)(nowUs - prevArrivalUs) : 0;
    const bool gap = (prevArrivalUs != 0 && arrivalGapUs > OPFLOW_DRONECAN_GAP_US) ||
                     (intervalUs > OPFLOW_DRONECAN_MAX_INTERVAL_US);

    prevArrivalUs = nowUs;

    if (gap) {
        /*
         * Drop this sample. Also drop the gyro accumulated across the hole,
         * so the next sample is not compensated with rotation from the gap.
         * The position estimator still uses the wall-clock time between
         * accepted samples; that dt is not clamped here.
         */
        opflowZeroGyroAccumulator();
        return;
    }

    ATOMIC_BLOCK(NVIC_PRIO_CAN) {
        acc.sumFlow[0] += msg->flow_integral[0];
        acc.sumFlow[1] += msg->flow_integral[1];
        acc.sumIntervalUs += intervalUs;
        if (msg->quality < acc.minQuality) {
            acc.minQuality = msg->quality;
        }
        acc.count++;
    }
}

virtualOpflowVTable_t opflowDronecanVtable = {
    .detect = dronecanOpflowDetect,
    .init = dronecanOpflowInit,
    .update = dronecanOpflowUpdate
};

#endif
