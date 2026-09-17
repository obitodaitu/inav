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
 * This file is free software: you may copy, redistribute and/or modify it
 * under the terms of the GNU General Public License as published by the
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

#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * Generic Unreal Engine simulator backend.
 *
 * Unlike the X-Plane backend, which collapses every motor into a single
 * throttle value and only exposes three control surfaces, this backend
 * transports up to UE_MAX_MOTORS independent motor outputs and up to
 * UE_MAX_SERVOS control surfaces. That is what makes multirotors flyable.
 *
 * The wire format is the X-Plane RREF/DREF UDP datagram format so that the
 * existing framing helpers and the "reply to source address" behaviour can be
 * reused, but the dataref namespace is independent (inav_ue/...) and the
 * protocol is versioned via inav_ue/plugin/protocolVersion.
 *
 * ---------------------------------------------------------------------------
 * Wire convention (validated empirically against the motor mixer)
 * ---------------------------------------------------------------------------
 * Every simulator -> SITL dataref is a plain aerospace value in the aircraft's
 * own frame. A simulator that produces standard FRD body rates and standard
 * ZYX Euler angles needs no sign fixups at all:
 *
 *   attitude/roll     deg, positive = roll right (right wing down)
 *   attitude/pitch    deg, positive = nose UP
 *   attitude/yaw      deg, positive = compass heading (0..360 clockwise from N)
 *
 *   gyro/x            deg/s, positive = rolling right
 *   gyro/y            deg/s, positive = pitching nose UP
 *   gyro/z            deg/s, positive = yawing right (clockwise seen from above)
 *
 *   accel/x,y,z       g, the negative of the specific force in the FRD body
 *                     frame (forward, right, down). At rest that is
 *                     (0, 0, +1); level flight accelerating forward at 1 g
 *                     reads (-1, 0, +1). exchangeDataWithUe() negates it back
 *                     for INAV's FLU accelerometer.
 *
 *   gps/...           deg / m / m/s / count as named; heading is degrees
 *   baro/pressure     Pa,  baro/temperature deg C
 *   sensors/airspeed  m/s, sensors/mag* gauss, sensors/rangefinder cm
 *   sensors/batteryVoltage V, sensors/batteryCurrent A
 *   rc/rssi           0..999, rc/failsafe and rc/valid 0 or 1
 *   rc/channel[n]     -1..+1, same sense as an RC stick after trimming
 *
 * SITL -> simulator DREF writes:
 *   actuators/motor[n]        0..1
 *   actuators/servo[n]        -1..+1
 *   status/armed              0 or 1
 *   status/flightMode         flightModeFlags_e
 *   status/armingDisabledReason  armingDisableFlags_e
 *   plugin/heartbeat          1
 *   plugin/sitlVersion        version string, sent once at connect
 *
 * The internal INAV conventions these map onto, for reference: roll is
 * positive to the right, pitch is positive NOSE DOWN, yaw angle is a compass
 * heading, yaw rate is positive to the LEFT and the accelerometer is a
 * specific force in FLU. That is why ue.c negates pitch and gyro Y/Z when
 * injecting.
 */

#define UE_MAX_MOTORS   4
#define UE_MAX_SERVOS   4
#define UE_MAX_PWM_OUTS (UE_MAX_MOTORS + UE_MAX_SERVOS)

bool simUeInit(char* ip, int port, uint8_t* mapping, uint8_t mapCount, bool imu);
void simUeClose(void);
