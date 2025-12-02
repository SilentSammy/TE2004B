/*
 * sensors.c
 *
 *  Created on: Nov 28, 2025
 *      Author: abrah
 */

#include "sensors.h"
#include "can.h"
#include <string.h>

extern ControlMode currentMode;

int32_t readEncoder(void)
{
    static int32_t lastPosition = 0;

    // Get the encoder message from latestMsgs array
    CANMessage* encoderMsg = getCANMessageByID(0x123);
    if (encoderMsg != NULL && encoderMsg->length >= 4) {
        memcpy(&lastPosition, encoderMsg->data, sizeof(lastPosition));
    }

    return lastPosition;
}

bool readMotorControl(MotorControl* control)
{
    static MotorControl lastControl = {0.0f, 0.0f, 0.0f};
    static MotorControl previousControl = {0.0f, 0.0f, 0.0f};

    if (control == NULL) {
        return false;  // Invalid pointer
    }

    // Get the motor control message from latestMsgs array
    CANMessage* controlMsg = getCANMessageByID(0x125);
    if (controlMsg != NULL && controlMsg->length >= 6) {
        // Decode throttle (bytes 0-1), steering (bytes 2-3), omega (bytes 4-5)
        int16_t throttleRaw = (int16_t)((uint16_t)controlMsg->data[0] | ((uint16_t)controlMsg->data[1] << 8));
        int16_t steeringRaw = (int16_t)((uint16_t)controlMsg->data[2] | ((uint16_t)controlMsg->data[3] << 8));
        int16_t omegaRaw = (int16_t)((uint16_t)controlMsg->data[4] | ((uint16_t)controlMsg->data[5] << 8));

        // Convert to normalized float (-1.0 to +1.0)
        lastControl.throttle = throttleRaw / 1000.0f;
        lastControl.throttle *= 2;
        lastControl.steering = steeringRaw / 1000.0f;
        lastControl.omega = omegaRaw / 1000.0f;

        // Check if control values changed - switch to MANUAL mode if so
        if (lastControl.throttle != previousControl.throttle ||
            lastControl.steering != previousControl.steering ||
            lastControl.omega != previousControl.omega) {
            currentMode = MODE_MANUAL;
            previousControl = lastControl;
        }
    }

    // Copy to output
    *control = lastControl;

    // Return true if we have valid data (timestamp > 0 means we received at least one message)
    return (controlMsg != NULL && controlMsg->timestamp > 0);
}

bool readWaypoint2D(Waypoint2D* waypoint)
{
    static Waypoint2D lastWaypoint = {0.0f, 0.0f, 0.0f};
    static Waypoint2D previousWaypoint = {0.0f, 0.0f, 0.0f};

    if (waypoint == NULL) {
        return false;  // Invalid pointer
    }

    // Get the waypoint message from latestMsgs array
    CANMessage* waypointMsg = getCANMessageByID(0x129);
    if (waypointMsg != NULL && waypointMsg->length >= 6) {
        // Decode x (bytes 0-1), y (bytes 2-3), omega (bytes 4-5) - little-endian
        int16_t x_mm = (int16_t)((uint16_t)waypointMsg->data[0] | ((uint16_t)waypointMsg->data[1] << 8));
        int16_t y_mm = (int16_t)((uint16_t)waypointMsg->data[2] | ((uint16_t)waypointMsg->data[3] << 8));
        int16_t omega_dec = (int16_t)((uint16_t)waypointMsg->data[4] | ((uint16_t)waypointMsg->data[5] << 8));

        // Convert to float (mm → cm, decidegrees → degrees)
        lastWaypoint.x = x_mm / 10.0f;
        lastWaypoint.y = y_mm / 10.0f;
        lastWaypoint.omega = omega_dec / 10.0f;

        // Check if waypoint values changed - switch to WAYPOINT mode if so
        if (lastWaypoint.x != previousWaypoint.x ||
            lastWaypoint.y != previousWaypoint.y ||
            lastWaypoint.omega != previousWaypoint.omega) {
            currentMode = MODE_WAYPOINT;
            previousWaypoint = lastWaypoint;
        }
    }

    // Copy to output
    *waypoint = lastWaypoint;

    // Return true if we have valid data
    return (waypointMsg != NULL && waypointMsg->timestamp > 0);
}

bool readPseudoGPS(PseudoGPS* gps)
{
    static PseudoGPS lastGPS = {0.0f, 0.0f, 0.0f};

    if (gps == NULL) {
        return false;
    }

    // Get the pseudo-GPS message from latestMsgs array
    CANMessage* gpsMsg = getCANMessageByID(0x128);
    if (gpsMsg != NULL && gpsMsg->length >= 8) {
        // Decode big-endian format (MSB first)
        // X Position: bytes 2-3
        uint16_t x_raw = ((uint16_t)gpsMsg->data[2] << 8) | (uint16_t)gpsMsg->data[3];
        // Y Position: bytes 4-5
        uint16_t y_raw = ((uint16_t)gpsMsg->data[4] << 8) | (uint16_t)gpsMsg->data[5];
        // Width: bytes 6-7 (using as omega/angle)
        uint16_t w_raw = ((uint16_t)gpsMsg->data[6] << 8) | (uint16_t)gpsMsg->data[7];

        // Convert to float (pixels or appropriate units)
        lastGPS.x = (float)x_raw;
        lastGPS.y = (float)y_raw;
        lastGPS.w = (float)w_raw;
    }

    *gps = lastGPS;
    return (gpsMsg != NULL && gpsMsg->timestamp > 0);
}
