/*
 * robot_types.h
 *
 *  Created on: Nov 28, 2025
 *      Author: abrah
 */

#ifndef INC_ROBOT_TYPES_H_
#define INC_ROBOT_TYPES_H_

#include <stdint.h>

typedef struct {
    float throttle;  // Throttle value (-1.0 to +1.0)
    float steering;  // Steering value (-1.0 to +1.0)
    float omega;     // Omega (angular velocity) value
} MotorControl;

typedef struct {
    float x;     // X position in meters
    float y;     // Y position in meters
    float w; // Angle (omega) in degrees
} PseudoGPS;

typedef struct {
    float x;            // Target X coordinate
    float y;            // Target Y coordinate
    float omega;        // Target orientation in degrees
} Waypoint2D;

typedef enum {
    MODE_MANUAL,      // Direct control via throttle/steering/omega
    MODE_WAYPOINT     // Autonomous waypoint navigation
} ControlMode;

#endif /* INC_ROBOT_TYPES_H_ */
