/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_GPS_LAP_TIMER

#include "drivers/time.h"

#include "io/gps.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "fc/gps_lap_timer.h"

PG_REGISTER_WITH_RESET_TEMPLATE(gpsLapTimerConfig_t, gpsLapTimerConfig, PG_GPS_LAP_TIMER, 1);

PG_RESET_TEMPLATE(gpsLapTimerConfig_t, gpsLapTimerConfig,
    .gateLat = 0,
    .gateLon = 0,
    .minimumLapTimeSeconds = 10,
    .gateToleranceCm = 500,
);

// if 1000 readings of 32-bit values are used, the total (for use in calculating the averate) would need 42 bits max.
static int64_t gateSetLatReadings;
static int64_t gateSetLonReadings;
static bool settingGate = false;
static uint32_t minDistance = UINT32_MAX;
static uint32_t minDistanceTime = 0L;
static uint32_t timeOfLastLap = 0L;
static bool timerRunning = false;
static bool wasInCircle = false;

gpsLapTimerData_t gpsLapTimerData;

void gpsLapTimerInit(void)
{
    gpsLapTimerData.gateLocation.lat = gpsLapTimerConfig()->gateLat;
    gpsLapTimerData.gateLocation.lon  = gpsLapTimerConfig()->gateLon;
    gpsLapTimerData.currentLapTime = 0;
    gpsLapTimerData.numberOfSetReadings = 0;
    gpsLapTimerData.bestLapTime = 0;
    gpsLapTimerData.best3Consec = 0;
    gpsLapTimerData.distToPoint = 0;
    gpsLapTimerData.previousLaps[0] = 0;
    gpsLapTimerData.previousLaps[1] = 0;
    gpsLapTimerData.previousLaps[2] = 0;
    timerRunning = false;
}

void gpsLapTimerStartSetGate(void)
{
    settingGate = true;
    gateSetLatReadings = 0;
    gateSetLonReadings = 0;
    gpsLapTimerData.numberOfSetReadings = 0;
}

void gpsLapTimerProcessSettingGate(void)
{
    if (gpsLapTimerData.numberOfSetReadings < 1000){
        gateSetLatReadings += gpsSol.llh.lat;
        gateSetLonReadings += gpsSol.llh.lon;
        gpsLapTimerData.numberOfSetReadings++;
    }
    int32_t newLat = gateSetLatReadings / gpsLapTimerData.numberOfSetReadings;
    int32_t newLon = gateSetLonReadings / gpsLapTimerData.numberOfSetReadings;
    int32_t bearing;
    GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &newLat, &newLon, &gpsLapTimerData.distToPoint, &bearing);
}

void gpsLapTimerEndSetGate(void)
{
    settingGate = false;

    int32_t newLat = gateSetLatReadings / gpsLapTimerData.numberOfSetReadings;
    int32_t newLon = gateSetLonReadings / gpsLapTimerData.numberOfSetReadings;

    gpsLapTimerConfigMutable()->gateLat = newLat;
    gpsLapTimerConfigMutable()->gateLon = newLon;
    gpsLapTimerData.gateLocation.lat = gpsLapTimerConfig()->gateLat;
    gpsLapTimerData.gateLocation.lon = gpsLapTimerConfig()->gateLon;
}

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

void gpsLapTimerUpdate(void)
{
    uint32_t currentTime = millis();

    if (timerRunning) {
        gpsLapTimerData.currentLapTime = currentTime - timeOfLastLap;
    } else {
        gpsLapTimerData.currentLapTime = 0;
    }

    // Current lap time is at least the min lap timer or timer not running
    if (gpsLapTimerData.currentLapTime > (gpsLapTimerConfig()->minimumLapTimeSeconds * 1000) || !timerRunning) {

        // Within radius of gate
        uint32_t distance;
        int32_t bearing;
        GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &gpsLapTimerData.gateLocation.lat, &gpsLapTimerData.gateLocation.lon, &distance, &bearing);
        if (distance < gpsLapTimerConfig()->gateToleranceCm) {
            if (!wasInCircle) { // Just entered the circle
                minDistance = UINT32_MAX;
            }
            if (distance < minDistance) {
                minDistance = distance;
                minDistanceTime = currentTime;
            }
            wasInCircle = true;
        } else {
            wasInCircle = false;
            if (wasInCircle) { // Just left the circle
                if (timerRunning) { // Not the first time through the gate
                    uint32_t lapTime = minDistanceTime - timeOfLastLap;
                    // Update best 3 consecutive
                    gpsLapTimerData.previousLaps[2] = gpsLapTimerData.previousLaps[1];
                    gpsLapTimerData.previousLaps[1] = gpsLapTimerData.previousLaps[0];
                    gpsLapTimerData.previousLaps[0] = lapTime;
                    if (gpsLapTimerData.previousLaps[2] != 0 && gpsLapTimerData.previousLaps[1] != 0 && gpsLapTimerData.previousLaps[0] != 0 &&
                        (gpsLapTimerData.previousLaps[2] + gpsLapTimerData.previousLaps[1] + gpsLapTimerData.previousLaps[0] < gpsLapTimerData.best3Consec || gpsLapTimerData.best3Consec == 0)) {
                        gpsLapTimerData.best3Consec = gpsLapTimerData.previousLaps[0] + gpsLapTimerData.previousLaps[1] + gpsLapTimerData.previousLaps[2];
                    }
                    // Update best lap time
                    if (gpsLapTimerData.previousLaps[0] != 0 &&
                        (gpsLapTimerData.previousLaps[0] < gpsLapTimerData.bestLapTime || gpsLapTimerData.bestLapTime == 0)) {
                        gpsLapTimerData.bestLapTime = gpsLapTimerData.previousLaps[0];
                    }
                }
                timeOfLastLap = minDistanceTime;
                timerRunning = true;
            }
        }
    }

    if (settingGate) {
        gpsLapTimerProcessSettingGate();
    }
}

#endif // GPS_LAP_TIMER
