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

/*
 * Generic Unreal Engine simulator backend.
 *
 * Wire format is the X-Plane RREF/DREF datagram format, so the same framing
 * rules apply:
 *
 *   subscription (SITL -> sim)  413 bytes:
 *       "RREF" + 1 pad + int32 LE frequency + int32 LE dataref index
 *       + NUL terminated dataref name, zero padded
 *   response (sim -> SITL)      <= 1024 bytes:
 *       "RREF" + 1 pad + N * { int32 LE index, float32 LE value }
 *       and it MUST be sent back to the source address the subscription
 *       came from, because SITL never binds a port.
 *   write (SITL -> sim)         509 bytes:
 *       "DREF" + 1 pad + float32 LE value + NUL terminated name,
 *       space padded
 *
 * The dataref namespace and the semantics are our own (see the table below),
 * versioned through inav_ue/plugin/protocolVersion. In particular the motor
 * outputs are NOT collapsed into a single throttle value.
 */

#include "target/SITL/sim/ue.h"

#include <arpa/inet.h>
#include <errno.h>
#include <math.h>
#include <netdb.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "build/version.h"
#include "common/maths.h"
#include "common/utils.h"
#include "drivers/accgyro/accgyro_fake.h"
#include "drivers/barometer/barometer_fake.h"
#include "drivers/compass/compass_fake.h"
#include "drivers/pitotmeter/pitotmeter_fake.h"
#include "drivers/pwm_mapping.h"
#include "drivers/rangefinder/rangefinder_virtual.h"
#include "drivers/time.h"
#include "fc/runtime_config.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "io/gps.h"
#include "io/rangefinder.h"
#include "platform.h"
#include "rx/sim.h"
#include "sensors/acceleration.h"
#include "sensors/battery_sensor_fake.h"
#include "target.h"
#include "target/SITL/sim/simHelper.h"

#define UE_PORT                 49000
#define UE_PROTOCOL_VERSION     2
#define UE_HANDSHAKE_TIMEOUT_MS 10000
#define UE_RECV_TIMEOUT_MS      500
#define UE_SIM_LOST_MS          1000
#define UE_SUBSCRIBE_HZ         100
#define UE_RC_CHANNEL_COUNT     8

typedef enum
{
    UE_DISCONNECTED,
    UE_CONNECTING,
    UE_INIT,
    UE_CONNECTED,
} ueState_t;

/*
 * Dataref indices exchanged as part of the aggregated RREF response.
 * Index 0 is the handshake: the simulator reports the protocol version it
 * speaks and SITL refuses to connect if it is older than UE_PROTOCOL_VERSION.
 */
typedef enum
{
    UE_DREF_PROTOCOL_VERSION = 0,

    UE_DREF_ATTITUDE_ROLL,
    UE_DREF_ATTITUDE_PITCH,
    UE_DREF_ATTITUDE_YAW,

    UE_DREF_GYRO_X,
    UE_DREF_GYRO_Y,
    UE_DREF_GYRO_Z,

    UE_DREF_ACCEL_X,
    UE_DREF_ACCEL_Y,
    UE_DREF_ACCEL_Z,

    UE_DREF_GPS_LATITUDE,
    UE_DREF_GPS_LONGITUDE,
    UE_DREF_GPS_ELEVATION,
    UE_DREF_GPS_GROUNDSPEED,
    UE_DREF_GPS_HEADING,
    UE_DREF_GPS_NUM_SATS,
    UE_DREF_GPS_FIX_TYPE,
    UE_DREF_GPS_VEL_NORTH,
    UE_DREF_GPS_VEL_EAST,
    UE_DREF_GPS_VEL_DOWN,

    UE_DREF_BARO_PRESSURE,
    UE_DREF_BARO_TEMPERATURE,

    UE_DREF_AIRSPEED,

    UE_DREF_MAG_X,
    UE_DREF_MAG_Y,
    UE_DREF_MAG_Z,

    UE_DREF_RANGEFINDER,

    UE_DREF_BATTERY_VOLTAGE,
    UE_DREF_BATTERY_CURRENT,

    UE_DREF_RSSI,
    UE_DREF_FAILSAFE,
    UE_DREF_RC_VALID,
    UE_DREF_RC_CHANNEL_1,

    UE_DREF_RC_CHANNEL_8 = UE_DREF_RC_CHANNEL_1 + UE_RC_CHANNEL_COUNT - 1,

    /* Simulator sample time, sent as four 16-bit chunks of the int64 Unix epoch
     * nanoseconds the plugin stamps each frame with. ns0 is the least
     * significant chunk and every chunk is an exact float32. Appended here so
     * that every existing index stays put: the plugin's table mirrors this enum
     * index for index. */
    UE_DREF_TIME_NS_0,
    UE_DREF_TIME_NS_1,
    UE_DREF_TIME_NS_2,
    UE_DREF_TIME_NS_3,

    UE_DREF_COUNT,
} ueDref_t;

static const char * const ueDrefNames[UE_DREF_COUNT] = {
    [UE_DREF_PROTOCOL_VERSION]  = "inav_ue/plugin/protocolVersion",

    [UE_DREF_ATTITUDE_ROLL]     = "inav_ue/attitude/roll",
    [UE_DREF_ATTITUDE_PITCH]    = "inav_ue/attitude/pitch",
    [UE_DREF_ATTITUDE_YAW]      = "inav_ue/attitude/yaw",

    [UE_DREF_GYRO_X]            = "inav_ue/gyro/x",
    [UE_DREF_GYRO_Y]            = "inav_ue/gyro/y",
    [UE_DREF_GYRO_Z]            = "inav_ue/gyro/z",

    [UE_DREF_ACCEL_X]           = "inav_ue/accel/x",
    [UE_DREF_ACCEL_Y]           = "inav_ue/accel/y",
    [UE_DREF_ACCEL_Z]           = "inav_ue/accel/z",

    [UE_DREF_GPS_LATITUDE]      = "inav_ue/gps/latitude",
    [UE_DREF_GPS_LONGITUDE]     = "inav_ue/gps/longitude",
    [UE_DREF_GPS_ELEVATION]     = "inav_ue/gps/elevation",
    [UE_DREF_GPS_GROUNDSPEED]   = "inav_ue/gps/groundspeed",
    [UE_DREF_GPS_HEADING]       = "inav_ue/gps/heading",
    [UE_DREF_GPS_NUM_SATS]      = "inav_ue/gps/numSats",
    [UE_DREF_GPS_FIX_TYPE]      = "inav_ue/gps/fix",
    [UE_DREF_GPS_VEL_NORTH]     = "inav_ue/gps/velNorth",
    [UE_DREF_GPS_VEL_EAST]      = "inav_ue/gps/velEast",
    [UE_DREF_GPS_VEL_DOWN]      = "inav_ue/gps/velDown",

    [UE_DREF_BARO_PRESSURE]     = "inav_ue/baro/pressure",
    [UE_DREF_BARO_TEMPERATURE]  = "inav_ue/baro/temperature",

    [UE_DREF_AIRSPEED]          = "inav_ue/sensors/airspeed",

    [UE_DREF_MAG_X]             = "inav_ue/sensors/magX",
    [UE_DREF_MAG_Y]             = "inav_ue/sensors/magY",
    [UE_DREF_MAG_Z]             = "inav_ue/sensors/magZ",

    [UE_DREF_RANGEFINDER]       = "inav_ue/sensors/rangefinder",

    [UE_DREF_BATTERY_VOLTAGE]   = "inav_ue/sensors/batteryVoltage",
    [UE_DREF_BATTERY_CURRENT]   = "inav_ue/sensors/batteryCurrent",

    [UE_DREF_RSSI]              = "inav_ue/rc/rssi",
    [UE_DREF_FAILSAFE]          = "inav_ue/rc/failsafe",
    [UE_DREF_RC_VALID]          = "inav_ue/rc/valid",

    [UE_DREF_RC_CHANNEL_1]      = "inav_ue/rc/channel[0]",
    [UE_DREF_RC_CHANNEL_1 + 1]  = "inav_ue/rc/channel[1]",
    [UE_DREF_RC_CHANNEL_1 + 2]  = "inav_ue/rc/channel[2]",
    [UE_DREF_RC_CHANNEL_1 + 3]  = "inav_ue/rc/channel[3]",
    [UE_DREF_RC_CHANNEL_1 + 4]  = "inav_ue/rc/channel[4]",
    [UE_DREF_RC_CHANNEL_1 + 5]  = "inav_ue/rc/channel[5]",
    [UE_DREF_RC_CHANNEL_1 + 6]  = "inav_ue/rc/channel[6]",
    [UE_DREF_RC_CHANNEL_1 + 7]  = "inav_ue/rc/channel[7]",

    [UE_DREF_TIME_NS_0]         = "inav_ue/time/ns0",
    [UE_DREF_TIME_NS_1]         = "inav_ue/time/ns1",
    [UE_DREF_TIME_NS_2]         = "inav_ue/time/ns2",
    [UE_DREF_TIME_NS_3]         = "inav_ue/time/ns3",
};

static const char * const ueMotorDrefNames[UE_MAX_MOTORS] = {
    "inav_ue/actuators/motor[0]",
    "inav_ue/actuators/motor[1]",
    "inav_ue/actuators/motor[2]",
    "inav_ue/actuators/motor[3]",
};

static const char * const ueServoDrefNames[UE_MAX_SERVOS] = {
    "inav_ue/actuators/servo[0]",
    "inav_ue/actuators/servo[1]",
    "inav_ue/actuators/servo[2]",
    "inav_ue/actuators/servo[3]",
};

#define UE_DREF_STATUS_ARMED        "inav_ue/status/armed"
#define UE_DREF_STATUS_FLIGHT_MODE  "inav_ue/status/flightMode"
#define UE_DREF_STATUS_ARM_BLOCKER  "inav_ue/status/armingDisabledReason"
#define UE_DREF_PLUGIN_HEARTBEAT    "inav_ue/plugin/heartbeat"
#define UE_DREF_PLUGIN_SITL_VERSION "inav_ue/plugin/sitlVersion"

static _Atomic ueState_t connectionState = UE_DISCONNECTED;

static int8_t motorSlotToMotorIndex[UE_MAX_MOTORS];
static int8_t servoSlotToServoIndex[UE_MAX_SERVOS];

static struct sockaddr_storage serverAddr;
static socklen_t serverAddrLen;
static int sockFd;
static pthread_t listenThread = 0;
static bool useImu = false;

// Sensors reported by the simulator
static float latitude = 0;
static float longitude = 0;
static float gpsElevation = 0;
static float groundspeed = 0;
static float heading = 0;
static float velNorth = 0;
static float velEast = 0;
static float velDown = 0;
static int numSats = 16;
static int fixType = GPS_FIX_3D;
static float roll = 0;
static float pitch = 0;
static float yaw = 0;
static float accelX = 0;
static float accelY = 0;
static float accelZ = 0;
static float gyroX = 0;
static float gyroY = 0;
static float gyroZ = 0;
static float baroPressure = 101325.0f;
static float baroTemperature = 15.0f;
static float airspeed = 0;
static float magX = 0;
static float magY = 0;
static float magZ = 0;
static int32_t rangefinderAltitude = -1;    // cm
static float batteryVoltage = 22.2f;        // 6S nominal
static float batteryCurrent = 0;
static uint16_t rssi = 0;
static bool failsafe = false;
static bool rcValid = false;

/* Last received sample time, as four 16-bit chunks of the plugin's int64. */
static uint16_t simTimeChunk[4] = { 0, 0, 0, 0 };
static float rcChannels[UE_RC_CHANNEL_COUNT];

// Liveness tracking
static timeMs_t lastPacketMs = 0;
static bool simLost = false;

// Byte order helpers, mirroring xplane.c. Kept static so that linking both
// backends into the same binary does not produce duplicate symbols.
static uint32_t xint2uint32(uint8_t* buf)
{
    return (uint32_t)buf[3] << 24 | (uint32_t)buf[2] << 16 | (uint32_t)buf[1] << 8 | (uint32_t)buf[0];
}

static float xflt2float(uint8_t* buf)
{
    union
    {
        float f;
        uint32_t i;
    } v;

    v.i = xint2uint32(buf);
    return v.f;
}

static float clampf(float value, float min, float max)
{
    return value < min ? min : (value > max ? max : value);
}

static bool registerDref(ueDref_t id, const char* dref, uint32_t freq)
{
    char buf[413];
    memset(buf, 0, sizeof(buf));

    strcpy(buf, "RREF");
    memcpy(buf + 5, &freq, 4);
    memcpy(buf + 9, &id, 4);
    memcpy(buf + 13, dref, strlen(dref) + 1);

    ssize_t sentBytes = sendto(sockFd, (void*)buf, sizeof(buf), 0, (struct sockaddr*)&serverAddr, serverAddrLen);
    return (sentBytes >= 0);
}

/*
 * Best effort write. Liveness is decided by inbound traffic, not by the
 * result of sendto(), so that a simulator that goes away and comes back is
 * picked up again instead of terminating the backend.
 */
static void sendDref(const char* dref, float value)
{
    char buf[509];
    strcpy(buf, "DREF");
    memcpy(buf + 5, &value, 4);
    memset(buf + 9, ' ', sizeof(buf) - 9);
    strcpy(buf + 9, dref);

    sendto(sockFd, (void*)buf, sizeof(buf), 0, (struct sockaddr*)&serverAddr, serverAddrLen);
}

static void registerAllDrefs(uint32_t freq)
{
    for (int i = 0; i < UE_DREF_COUNT; i++) {
        registerDref((ueDref_t)i, ueDrefNames[i], freq);
    }
}

static bool receiveSingleDref(ueDref_t inDref, float* value)
{
    uint8_t buf[1024];
    struct sockaddr_storage remoteAddr;
    socklen_t slen = sizeof(remoteAddr);
    int recvLen;

    recvLen = recvfrom(sockFd, buf, sizeof(buf), 0, (struct sockaddr*)&remoteAddr, &slen);
    if (recvLen < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
        return false;
    }

    if (recvLen < 5 || strncmp((char*)buf, "RREF", 4) != 0) {
        return false;
    }

    // Bounds safe: stop unless a full index/value pair fits.
    for (int i = 5; i + 8 <= recvLen; i += 8) {
        if ((ueDref_t)xint2uint32(&buf[i]) == inDref) {
            *value = xflt2float(&(buf[i + 4]));
            return true;
        }
    }

    return false;
}

/*
 * Push the actuator state to the simulator and pull a fresh sensor frame in.
 * Returns true when a usable RREF frame was received.
 */
static bool exchangeDataWithUe(void)
{
    uint8_t buf[1024];
    struct sockaddr_storage remoteAddr;
    socklen_t slen = sizeof(remoteAddr);
    int recvLen;

    // ---- Actuators: every motor gets its own channel ----
    for (int slot = 0; slot < UE_MAX_MOTORS; slot++) {
        float value = 0.0f;
        const int motorIndex = motorSlotToMotorIndex[slot];
        if (motorIndex >= 0 && motorIndex < MAX_MOTORS) {
            value = PWM_TO_FLOAT_0_1(motor[motorIndex]);
        }
        sendDref(ueMotorDrefNames[slot], value);
    }

    for (int slot = 0; slot < UE_MAX_SERVOS; slot++) {
        float value = 0.0f;
        const int servoIndex = servoSlotToServoIndex[slot];
        if (servoIndex >= 0 && servoIndex < MAX_SERVOS) {
            value = PWM_TO_FLOAT_MINUS_1_1(servo[servoIndex]);
        }
        sendDref(ueServoDrefNames[slot], value);
    }

    sendDref(UE_DREF_PLUGIN_HEARTBEAT, 1.0f);
    sendDref(UE_DREF_STATUS_ARMED, ARMING_FLAG(ARMED) ? 1.0f : 0.0f);
    sendDref(UE_DREF_STATUS_FLIGHT_MODE, (float)getFlightModeForTelemetry());
    // Single bit value, so it survives the float round trip exactly.
    sendDref(UE_DREF_STATUS_ARM_BLOCKER, (float)isArmingDisabledReason());

    // ---- Sensors ----
    recvLen = recvfrom(sockFd, buf, sizeof(buf), 0, (struct sockaddr*)&remoteAddr, &slen);
    if (recvLen < 0) {
        return false;
    }
    if (recvLen < 5 || strncmp((char*)buf, "RREF", 4) != 0) {
        return false;
    }

    bool gotData = false;
    for (int i = 5; i + 8 <= recvLen; i += 8) {
        const uint32_t dref = xint2uint32(&buf[i]);
        if (dref >= UE_DREF_COUNT) {
            continue;
        }

        const float value = xflt2float(&buf[i + 4]);
        gotData = true;

        switch (dref) {
            case UE_DREF_GPS_LATITUDE:
                latitude = value;
                break;

            case UE_DREF_GPS_LONGITUDE:
                longitude = value;
                break;

            case UE_DREF_GPS_ELEVATION:
                gpsElevation = value;
                break;

            case UE_DREF_GPS_GROUNDSPEED:
                groundspeed = value;
                break;

            case UE_DREF_GPS_HEADING:
                heading = value;
                break;

            case UE_DREF_GPS_NUM_SATS:
                numSats = (int)value;
                break;

            case UE_DREF_GPS_FIX_TYPE:
                fixType = (int)value;
                break;

            case UE_DREF_GPS_VEL_NORTH:
                velNorth = value;
                break;

            case UE_DREF_GPS_VEL_EAST:
                velEast = value;
                break;

            case UE_DREF_GPS_VEL_DOWN:
                velDown = value;
                break;

            case UE_DREF_ATTITUDE_ROLL:
                roll = value;
                break;

            case UE_DREF_ATTITUDE_PITCH:
                pitch = value;
                break;

            case UE_DREF_ATTITUDE_YAW:
                yaw = value;
                break;

            case UE_DREF_GYRO_X:
                gyroX = value;
                break;

            case UE_DREF_GYRO_Y:
                gyroY = value;
                break;

            case UE_DREF_GYRO_Z:
                gyroZ = value;
                break;

            case UE_DREF_ACCEL_X:
                accelX = value;
                break;

            case UE_DREF_ACCEL_Y:
                accelY = value;
                break;

            case UE_DREF_ACCEL_Z:
                accelZ = value;
                break;

            case UE_DREF_BARO_PRESSURE:
                baroPressure = value;
                break;

            case UE_DREF_BARO_TEMPERATURE:
                baroTemperature = value;
                break;

            case UE_DREF_AIRSPEED:
                airspeed = value;
                break;

            case UE_DREF_MAG_X:
                magX = value;
                break;

            case UE_DREF_MAG_Y:
                magY = value;
                break;

            case UE_DREF_MAG_Z:
                magZ = value;
                break;

            case UE_DREF_RANGEFINDER:
                rangefinderAltitude = (int32_t)value;
                break;

            case UE_DREF_BATTERY_VOLTAGE:
                batteryVoltage = value;
                break;

            case UE_DREF_BATTERY_CURRENT:
                batteryCurrent = value;
                break;

            case UE_DREF_RSSI:
                rssi = (uint16_t)value;
                break;

            case UE_DREF_FAILSAFE:
                failsafe = value >= 1.0f;
                break;

            case UE_DREF_RC_VALID:
                rcValid = value >= 0.5f;
                break;

            case UE_DREF_TIME_NS_0:
                simTimeChunk[0] = (uint16_t)value;
                break;

            case UE_DREF_TIME_NS_1:
                simTimeChunk[1] = (uint16_t)value;
                break;

            case UE_DREF_TIME_NS_2:
                simTimeChunk[2] = (uint16_t)value;
                break;

            case UE_DREF_TIME_NS_3:
                simTimeChunk[3] = (uint16_t)value;
                break;

            default:
                if (dref >= UE_DREF_RC_CHANNEL_1 && dref <= UE_DREF_RC_CHANNEL_8) {
                    rcChannels[dref - UE_DREF_RC_CHANNEL_1] = value;
                }
                break;
        }
    }

    if (!gotData) {
        return false;
    }

    /* Publish the simulator's own instant for this frame, so everything INAV
     * derives from it can be stamped with the clock the simulator and its
     * images share rather than with INAV's local one. */
    {
        const int64_t sampleNs =
            (int64_t)simTimeChunk[0] |
            ((int64_t)simTimeChunk[1] << 16) |
            ((int64_t)simTimeChunk[2] << 32) |
            ((int64_t)simTimeChunk[3] << 48);

        if (sampleNs != 0) {
            simTimeSetSample(sampleNs, micros());
        }
    }

    // Wire yaw and heading are degrees (see the convention table in ue.h), so
    // a full turn is 360 here. The 3600 in xplane.c only looks right because
    // that dataref never goes negative in practice.
    if (yaw < 0) {
        yaw += 360.0f;
    }

    if (heading < 0) {
        heading += 360.0f;
    }

    if (rcValid) {
        uint16_t channelValues[UE_RC_CHANNEL_COUNT];
        for (int i = 0; i < UE_RC_CHANNEL_COUNT; i++) {
            channelValues[i] = FLOAT_MINUS_1_1_TO_PWM(clampf(rcChannels[i], -1.0f, 1.0f));
        }
        rxSimSetChannelValue(channelValues, UE_RC_CHANNEL_COUNT);
    }

    gpsFakeSet(
        (gpsFixType_e)fixType,
        (uint8_t)numSats,
        (int32_t)roundf(latitude * 10000000.0f),
        (int32_t)roundf(longitude * 10000000.0f),
        (int32_t)roundf(gpsElevation * 100.0f),
        (int16_t)roundf(groundspeed * 100.0f),
        (int16_t)roundf(heading * 10.0f),
        (int16_t)roundf(velNorth * 100.0f),
        (int16_t)roundf(velEast * 100.0f),
        (int16_t)roundf(velDown * 100.0f),
        0
    );

    const int16_t rollInav = roll * 10.0f;
    const int16_t pitchInav = -pitch * 10.0f;
    const int16_t yawInav = yaw * 10.0f;

    if (!useImu) {
        imuSetAttitudeRPY(rollInav, pitchInav, yawInav);
        imuUpdateAttitude(micros());
    }

    fakeAccSet(
        constrainToInt16(-accelX * GRAVITY_MSS * 1000.0f),
        constrainToInt16(accelY * GRAVITY_MSS * 1000.0f),
        constrainToInt16(accelZ * GRAVITY_MSS * 1000.0f)
    );

    fakeGyroSet(
        constrainToInt16(gyroX * 16.0f),
        constrainToInt16(-gyroY * 16.0f),
        constrainToInt16(-gyroZ * 16.0f)
    );

    fakeBaroSet(
        (int32_t)roundf(baroPressure),
        (int32_t)DEGREES_TO_CENTIDEGREES(baroTemperature)
    );

    fakePitotSetAirspeed(airspeed * 100.0f);

    fakeMagSet(
        constrainToInt16(magX * 1024.0f),
        constrainToInt16(magY * 1024.0f),
        constrainToInt16(magZ * 1024.0f)
    );

    fakeBattSensorSetVbat((uint16_t)roundf(batteryVoltage * 100.0f));
    fakeBattSensorSetAmperage((uint16_t)roundf(batteryCurrent * 100.0f));
    rxSimSetRssi(rssi);
    rxSimSetFailsafe(failsafe);

    if (rangefinderAltitude > 0 && rangefinderAltitude <= RANGEFINDER_VIRTUAL_MAX_RANGE_CM) {
        fakeRangefindersSetData(rangefinderAltitude);
    } else {
        fakeRangefindersSetData(-1);
    }

    return true;
}

static void* listenWorker(void* arg)
{
    UNUSED(arg);

    while (connectionState != UE_DISCONNECTED) {
        switch (connectionState) {
            case UE_CONNECTING: {
                registerAllDrefs(0);            // clear stale subscriptions
                registerAllDrefs(UE_SUBSCRIBE_HZ);

                const timeMs_t deadline = millis() + UE_HANDSHAKE_TIMEOUT_MS;
                float version = 0;
                bool connected = false;

                while (!connected) {
                    if (receiveSingleDref(UE_DREF_PROTOCOL_VERSION, &version) &&
                        (int)version >= UE_PROTOCOL_VERSION) {
                        connected = true;
                    } else if (millis() >= deadline) {
                        fprintf(stderr, "[SIM] No UE simulator handshake within %d ms.\n",
                                UE_HANDSHAKE_TIMEOUT_MS);
                        connectionState = UE_DISCONNECTED;
                        break;
                    } else {
                        // The simulator may have been started after us
                        registerAllDrefs(UE_SUBSCRIBE_HZ);
                    }
                }

                if (connected) {
                    fprintf(stderr, "[SIM] UE simulator detected, protocol version %d.\n",
                            (int)version);
                    connectionState = UE_INIT;
                }
                break;
            }

            case UE_INIT: {
                // Prime the sensors once so that attitude, baro and GPS are
                // valid before the flight loop starts consuming them.
                exchangeDataWithUe();

                ENABLE_ARMING_FLAG(SIMULATOR_MODE_SITL);
                // The aircraft can wobble on the ground, which would prevent
                // accelerometer calibration from ever completing.
                ENABLE_STATE(ACCELEROMETER_CALIBRATED);

                lastPacketMs = millis();
                simLost = false;
                // Firmware version for the simulator's log, packed as
                // major + minor/100 + patch/10000 (9.1.0 -> 9.01). DREF only
                // carries floats, so the string form cannot go on the wire.
                sendDref(UE_DREF_PLUGIN_SITL_VERSION,
                         FC_VERSION_MAJOR + FC_VERSION_MINOR / 100.0f + FC_VERSION_PATCH_LEVEL / 10000.0f);
                connectionState = UE_CONNECTED;
                break;
            }

            case UE_CONNECTED: {
                if (exchangeDataWithUe()) {
                    lastPacketMs = millis();

                    if (simLost) {
                        simLost = false;
                        rxSimSetFailsafe(false);
                        fprintf(stderr, "[SIM] UE simulator reconnected.\n");
                    }

                    // Hand the main PID lock over exactly once per received
                    // frame. This is what paces the flight loop.
                    unlockMainPID();
                } else if (!simLost && (millis() - lastPacketMs) > UE_SIM_LOST_MS) {
                    simLost = true;
                    rxSimSetFailsafe(true);
                    fprintf(stderr, "[SIM] UE simulator lost, failsafe engaged.\n");
                    // Deliberately do NOT unlockMainPID(): with no fresh
                    // sensor data the flight loop must not run its attitude
                    // update. Failsafe is still acted upon by the rest of the
                    // loop, which zeroes the motor outputs.
                }
                break;
            }

            default:
                break;
        }
    }

    fprintf(stderr, "[SOCKET] UE connection closed.\n");
    return NULL;
}

bool simUeInit(char* ip, int port, uint8_t* mapping, uint8_t mapCount, bool imu)
{
    memset(motorSlotToMotorIndex, -1, sizeof(motorSlotToMotorIndex));
    memset(servoSlotToServoIndex, -1, sizeof(servoSlotToServoIndex));

    int motorSlot = 0;
    int servoSlot = 0;
    for (int i = 0; i < mapCount && i < UE_MAX_PWM_OUTS; i++) {
        if (mapping[i] & 0x80) {
            if (motorSlot < UE_MAX_MOTORS) {
                motorSlotToMotorIndex[motorSlot++] = mapping[i] & 0x7f;
            }
        } else {
            // parseMapping stores servo outputs one based, motors zero based
            const int servoIndex = (int)mapping[i] - 1;
            if (servoIndex >= 0 && servoIndex < MAX_SERVOS && servoSlot < UE_MAX_SERVOS) {
                servoSlotToServoIndex[servoSlot++] = servoIndex;
            }
        }
    }

    if (motorSlot == 0) {
        fprintf(stderr, "[SIM] No motor mapped, check --chanmap.\n");
        return false;
    }

    useImu = imu;
    connectionState = UE_CONNECTING;

    if (port == 0) {
        port = UE_PORT;  // use default port
    }

    if (lookupAddress(ip, port, SOCK_DGRAM, (struct sockaddr*)&serverAddr, &serverAddrLen) != 0) {
        return false;
    }

    sockFd = socket(((struct sockaddr*)&serverAddr)->sa_family, SOCK_DGRAM, IPPROTO_UDP);
    if (sockFd < 0) {
        return false;
    } else {
        char addrbuf[IPADDRESS_PRINT_BUFLEN];
        char* nptr = prettyPrintAddress((struct sockaddr*)&serverAddr, addrbuf, IPADDRESS_PRINT_BUFLEN);
        if (nptr != NULL) {
            fprintf(stderr, "[SOCKET] ue address = %s, fd=%d\n", nptr, sockFd);
        }
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = UE_RECV_TIMEOUT_MS * 1000;
    if (setsockopt(sockFd, SOL_SOCKET, SO_RCVTIMEO, (struct timeval*)&tv, sizeof(struct timeval))) {
        return false;
    }

    tv.tv_sec = 1;
    tv.tv_usec = 0;
    if (setsockopt(sockFd, SOL_SOCKET, SO_SNDTIMEO, (struct timeval*)&tv, sizeof(struct timeval))) {
        return false;
    }

    if (pthread_create(&listenThread, NULL, listenWorker, NULL) < 0) {
        return false;
    }

    while (connectionState != UE_CONNECTED && connectionState != UE_DISCONNECTED) {
        delay(50);
    }

    return connectionState == UE_CONNECTED;
}

void simUeClose(void)
{
    connectionState = UE_DISCONNECTED;  // signal worker to exit its loop
    close(sockFd);                      // unblock any pending recvfrom
    if (listenThread) {
        pthread_join(listenThread, NULL);
        listenThread = 0;
    }
    DISABLE_ARMING_FLAG(SIMULATOR_MODE_SITL);
}
