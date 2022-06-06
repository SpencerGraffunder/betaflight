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

#pragma once

#include "io/gps.h"

#include "pg/pg.h"

typedef enum {
    GATE_SIDE_LEFT  = 0,
    GATE_SIDE_RIGHT = 1,
} gpsLapTimerGateSide_e;

typedef struct gpsLapTimerData_s {
    gpsLocation_t gateLocationLeft;
    gpsLocation_t gateLocationRight;
    uint32_t previousLaps[3];
    uint32_t currentLapTime;
    uint16_t numberOfSetReadings;
    uint32_t bestLapTime;
    uint32_t best3Consec;
    uint32_t distToPoint;
} gpsLapTimerData_t;

typedef struct gpsLapTimerConfig_s {
    int32_t gateLeftLat;
    int32_t gateLeftLon;
    int32_t gateRightLat;
    int32_t gateRightLon;
    uint16_t minimumLapTimeSeconds;
} gpsLapTimerConfig_t;

PG_DECLARE(gpsLapTimerConfig_t, gpsLapTimerConfig);

extern gpsLapTimerData_t gpsLapTimerData;

void gpsLapTimerInit(void);
void gpsLapTimerUpdate(void);
void gpsLapTimerStartSetGate(void);
void gpsLapTimerEndSetGate(gpsLapTimerGateSide_e);
