/*
 * sensors.h
 *
 *  Created on: Nov 28, 2025
 *      Author: abrah
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "robot_types.h"
#include <stdbool.h>

int32_t readEncoder(void);
bool readMotorControl(MotorControl*);
bool readWaypoint2D(Waypoint2D*);
bool readPseudoGPS(PseudoGPS*);

#endif /* INC_SENSORS_H_ */
