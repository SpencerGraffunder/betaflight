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

#include "gps_lap_timer.h"

PG_REGISTER_WITH_RESET_TEMPLATE(gpsLapTimerConfig_t, gpsLapTimerConfig, PG_GPS_LAP_TIMER, 1);

PG_RESET_TEMPLATE(gpsLapTimerConfig_t, gpsLapTimerConfig,
    .gateLeftLat = 0,
    .gateLeftLon = 0,
    .gateRightLat = 0,
    .gateRightLon = 0,
    .minimumLapTimeSeconds = 10,
);

#define INTTYPE_SIGNED(T) ((T)-1 < (T)0)
#define INTTYPE_MAX(T)                      \
    (((T)1 << (8*sizeof(T)-INTTYPE_SIGNED(T)-1)) - 1 +   \
     ((T)1 << (8*sizeof(T)-INTTYPE_SIGNED(T)-1)))

// if 1000 readings of 32-bit values are used, the total (for use in calculating the averate) would need 42 bits max.
static int64_t gateSetLatReadings = 0;
static int64_t gateSetLonReadings = 0;
static bool settingGate = false;
static gpsLocation_t lastLocation;
static uint32_t lastLocationTime;
static uint32_t timeOfLastLap = 0L;
static bool timerRunning = false;

gpsLapTimerData_t gpsLapTimerData;

void gpsLapTimerInit(void)
{
    gpsLapTimerData.gateLocationLeft.lat = gpsLapTimerConfig()->gateLeftLat;
    gpsLapTimerData.gateLocationLeft.lon = gpsLapTimerConfig()->gateLeftLon;
    gpsLapTimerData.gateLocationRight.lat = gpsLapTimerConfig()->gateRightLat;
    gpsLapTimerData.gateLocationRight.lon  = gpsLapTimerConfig()->gateRightLon;
    gpsLapTimerData.currentLapTime = 0;
    gpsLapTimerData.numberOfSetReadings = 0;
    gpsLapTimerData.bestLapTime = 0;
    gpsLapTimerData.best3Consec = 0;
    gpsLapTimerData.distToPoint = 0;
    gpsLapTimerData.previousLaps[0] = 0;
    gpsLapTimerData.previousLaps[1] = 0;
    gpsLapTimerData.previousLaps[2] = 0;
    timerRunning = false;
    settingGate = false;
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

void gpsLapTimerEndSetGate(gpsLapTimerGateSide_e side)
{
    settingGate = false;

    int32_t newLat = gateSetLatReadings / gpsLapTimerData.numberOfSetReadings;
    int32_t newLon = gateSetLonReadings / gpsLapTimerData.numberOfSetReadings;

    if (side == GATE_SIDE_LEFT) {
        gpsLapTimerData.gateLocationLeft.lat = newLat;
        gpsLapTimerData.gateLocationLeft.lon = newLon;
        gpsLapTimerConfigMutable()->gateLeftLat = newLat;
        gpsLapTimerConfigMutable()->gateLeftLon = newLon;
    } else {
        gpsLapTimerConfigMutable()->gateRightLat = newLat;
        gpsLapTimerConfigMutable()->gateRightLon = newLon;
        gpsLapTimerData.gateLocationRight.lat = gpsLapTimerConfig()->gateRightLat;
        gpsLapTimerData.gateLocationRight.lon  = gpsLapTimerConfig()->gateRightLon;
    }
}

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

// These three functions taken from https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
// Given three collinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(gpsLocation_t p, gpsLocation_t q, gpsLocation_t r)
{
    if (q.lon <= MAX(p.lon, r.lon) && q.lon >= MIN(p.lon, r.lon) &&
        q.lat <= MAX(p.lat, r.lat) && q.lat >= MIN(p.lat, r.lat)) {
        return true;
    } else {
        return false;
    }
}
 
// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(gpsLocation_t p, gpsLocation_t q, gpsLocation_t r)
{
    int val = (q.lat - p.lat) * (r.lon - q.lon) -
              (q.lon - p.lon) * (r.lat - q.lat);
    if (val == 0) {
        return 0;  // collinear
    } else {
        return (val > 0) ? 1 : 2; // clock or counterclock wise
    }
}

// The main function that returns true if line segment 'gateLeftgateRight'
// and 'lastPoscurPos' intersect.
bool doSegmentsIntersect(gpsLocation_t gateLeft, gpsLocation_t gateRight, gpsLocation_t lastPos, gpsLocation_t curPos)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(gateLeft, gateRight, lastPos);
    int o2 = orientation(gateLeft, gateRight, curPos);
    int o3 = orientation(lastPos, curPos, gateLeft);
    int o4 = orientation(lastPos, curPos, gateRight);
 
    // General case
    if (o1 != o2 && o3 != o4) {
        return true;
    }

    // Special Cases
    // gateLeft, gateRight and lastPos are collinear and lastPos lies on segment gateLeftgateRight
    if (o1 == 0 && onSegment(gateLeft, lastPos, gateRight)) {
        return true;
    }
    // gateLeft, gateRight and curPos are collinear and curPos lies on segment gateLeftgateRight
    if (o2 == 0 && onSegment(gateLeft, curPos, gateRight)) {
        return true;
    }
    // lastPos, curPos and gateLeft are collinear and gateLeft lies on segment lastPoscurPos
    if (o3 == 0 && onSegment(lastPos, gateLeft, curPos)) {
        return true;
    }
     // lastPos, curPos and gateRight are collinear and gateRight lies on segment lastPoscurPos
    if (o4 == 0 && onSegment(lastPos, gateRight, curPos)) {
        return true;
    }
 
    return false; // Doesn't fall in any of the above cases
}

void lineLineIntersectionTime(gpsLocation_t *a, gpsLocation_t *b, gpsLocation_t *c, gpsLocation_t *d, uint32_t *t1, uint32_t *t2, uint32_t *intersection_time)
{ 
    // Line AB represented as a1x + b1y = c1
    int64_t a1 = b->lon - a->lon;
    int64_t b1 = a->lat - b->lat;
    int64_t c1 = a1 * a->lat + b1 * a->lon;
  
    // Line CD represented as a2x + b2y = c2
    int64_t a2 = d->lon - c->lon;
    int64_t b2 = c->lat - d->lat;
    int64_t c2 = a2 * c->lat + b2 * c->lon;
  
    int64_t determinant = a1 * b2 - a2 * b1;
  
    gpsLocation_t intersection;
    if (determinant == 0) {
        // The lines are parallel.
    } else {
        intersection.lat = (b2 * c1 - b1 * c2) / determinant;
        intersection.lon = (a1 * c2 - a2 * c1) / determinant;
    }

    int32_t dummy;
    uint32_t distCI;
    GPS_distance_cm_bearing(&c->lat, &c->lon, &intersection.lat, &intersection.lon, &distCI, &dummy);
    uint32_t distCD;
    GPS_distance_cm_bearing(&c->lat, &c->lon, &d->lat, &d->lon, &distCD, &dummy);
    *intersection_time = *t1 + (*t2 - *t1) * distCI / distCD;
    //cliPrintf("t1: %u, t2: %u, Intersect %u", *t1, *t2, *intersection_time);
}

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
        if (doSegmentsIntersect(gpsLapTimerData.gateLocationLeft, gpsLapTimerData.gateLocationRight, lastLocation, gpsSol.llh)) {
            uint32_t preciceTimeOfCrossing;
            uint32_t preciceLapTime = 0;
            //lineLineIntersectionTime(&gpsLapTimerData.gateLocationLeft, &gpsLapTimerData.gateLocationRight, &lastLocation, &gpsSol.llh, 
            //                        &lastLocationTime, &currentTime, &preciceTimeOfCrossing);
            preciceTimeOfCrossing = currentTime;
            if (timerRunning) {
                preciceLapTime = preciceTimeOfCrossing - timeOfLastLap;
            }
            timeOfLastLap = preciceTimeOfCrossing;
            // Update best 3 consecutive
            gpsLapTimerData.previousLaps[2] = gpsLapTimerData.previousLaps[1];
            gpsLapTimerData.previousLaps[1] = gpsLapTimerData.previousLaps[0];
            gpsLapTimerData.previousLaps[0] = preciceLapTime;
            if (gpsLapTimerData.previousLaps[2] != 0 && gpsLapTimerData.previousLaps[1] != 0 && gpsLapTimerData.previousLaps[0] != 0 &&
                (gpsLapTimerData.previousLaps[2] + gpsLapTimerData.previousLaps[1] + gpsLapTimerData.previousLaps[0] < gpsLapTimerData.best3Consec || gpsLapTimerData.best3Consec == 0)) {
                gpsLapTimerData.best3Consec = gpsLapTimerData.previousLaps[0] + gpsLapTimerData.previousLaps[1] + gpsLapTimerData.previousLaps[2];
            }
            // Update best lap time
            if (gpsLapTimerData.previousLaps[0] != 0 &&
                (gpsLapTimerData.previousLaps[0] < gpsLapTimerData.bestLapTime || gpsLapTimerData.bestLapTime == 0)) {
                gpsLapTimerData.bestLapTime = gpsLapTimerData.previousLaps[0];
            }
            timerRunning = true;
        }
    }

    lastLocation = gpsSol.llh;
    lastLocationTime = currentTime;

    if (settingGate) {
        gpsLapTimerProcessSettingGate();
    }
}

#endif // GPS_LAP_TIMER
