/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Runs the DroneCAN rangefinder through the real sensor layer:
 * rangefinderInit() -> dronecanRangefinderReceive() -> rangefinderProcess().
 */

#include <stdbool.h>
#include <stdint.h>

extern "C" {
    #include "platform.h"

    #include <dronecan_msgs.h>

    #include "drivers/time.h"

    #include "io/rangefinder.h"

    #include "scheduler/scheduler.h"

    #include "sensors/rangefinder.h"
    #include "sensors/sensors.h"

    static timeMs_t fakeMillis;
    static uint32_t sensorMask;

    uint8_t requestedSensors[SENSOR_INDEX_COUNT];
    uint8_t detectedSensors[SENSOR_INDEX_COUNT];

    timeMs_t millis(void) { return fakeMillis; }
    void rescheduleTask(cfTaskId_e taskId, timeDelta_t newPeriodUs) { (void)taskId; (void)newPeriodUs; }
    void sensorsSet(uint32_t mask) { sensorMask |= mask; }
    void sensorsClear(uint32_t mask) { sensorMask &= ~mask; }
    bool sensors(uint32_t mask) { return sensorMask & mask; }

    // Other drivers that sensors/rangefinder.c can select; never used here
    virtualRangefinderVTable_t rangefinderMSPVtable;
    virtualRangefinderVTable_t rangefinderBenewakeVtable;
    virtualRangefinderVTable_t rangefinderUSD1Vtable;
    virtualRangefinderVTable_t rangefinderNanoradarVtable;
    virtualRangefinderVTable_t rangefinderFakeVtable;
    bool vl53l0xDetect(rangefinderDev_t *dev) { (void)dev; return false; }
    bool vl53l1xDetect(rangefinderDev_t *dev) { (void)dev; return false; }
    bool us42Detect(rangefinderDev_t *dev) { (void)dev; return false; }
    bool tof10120Detect(rangefinderDev_t *dev) { (void)dev; return false; }
    bool teraRangerDetect(rangefinderDev_t *dev) { (void)dev; return false; }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

#define VALID_RANGE UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE
#define TOO_FAR     UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR

class DronecanRangefinderTest : public ::testing::Test {
protected:
    void SetUp() override {
        fakeMillis = 0;
        sensorMask = 0;
        rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_DRONECAN;
        rangefinderConfigMutable()->use_median_filtering = false;
    }
};

TEST_F(DronecanRangefinderTest, DetectedAndReportedAsPresent)
{
    EXPECT_TRUE(rangefinderInit());
    EXPECT_TRUE(sensors(SENSOR_RANGEFINDER));
    EXPECT_EQ(detectedSensors[SENSOR_INDEX_RANGEFINDER], RANGEFINDER_DRONECAN);
    EXPECT_EQ(rangefinderGetLatestRawAltitude(), RANGEFINDER_OUT_OF_RANGE);
}

TEST_F(DronecanRangefinderTest, ValidSampleReachesRawAndTiltCorrectedAltitude)
{
    ASSERT_TRUE(rangefinderInit());

    dronecanRangefinderReceive(1.216f, VALID_RANGE);
    rangefinderUpdate();
    EXPECT_TRUE(rangefinderProcess(1.0f));

    EXPECT_EQ(rangefinderGetLatestRawAltitude(), 122);
    EXPECT_EQ(rangefinderGetLatestAltitude(), 122);
    EXPECT_TRUE(rangefinderIsHealthy());

    // Same sample is not delivered twice
    EXPECT_FALSE(rangefinderProcess(1.0f));
    EXPECT_EQ(rangefinderGetLatestRawAltitude(), 122);
}

TEST_F(DronecanRangefinderTest, OutOfRangeSampleClearsAltitudeButStaysHealthy)
{
    ASSERT_TRUE(rangefinderInit());

    dronecanRangefinderReceive(1.0f, VALID_RANGE);
    rangefinderProcess(1.0f);
    fakeMillis = 400;
    dronecanRangefinderReceive(1.0f, TOO_FAR);
    EXPECT_TRUE(rangefinderProcess(1.0f));

    EXPECT_EQ(rangefinderGetLatestRawAltitude(), RANGEFINDER_OUT_OF_RANGE);
    EXPECT_EQ(rangefinderGetLatestAltitude(), RANGEFINDER_OUT_OF_RANGE);
    fakeMillis = 800;
    EXPECT_TRUE(rangefinderIsHealthy());
}

TEST_F(DronecanRangefinderTest, SilentSensorTimesOut)
{
    ASSERT_TRUE(rangefinderInit());

    dronecanRangefinderReceive(1.0f, VALID_RANGE);
    rangefinderProcess(1.0f);
    EXPECT_TRUE(rangefinderIsHealthy());

    fakeMillis = 499;
    EXPECT_FALSE(rangefinderProcess(1.0f));
    EXPECT_TRUE(rangefinderIsHealthy());

    fakeMillis = 500;
    EXPECT_FALSE(rangefinderIsHealthy());
}

TEST_F(DronecanRangefinderTest, NotDetectedWhenAnotherHardwareIsSelected)
{
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_NONE;

    EXPECT_FALSE(rangefinderInit());
    EXPECT_FALSE(sensors(SENSOR_RANGEFINDER));
}
