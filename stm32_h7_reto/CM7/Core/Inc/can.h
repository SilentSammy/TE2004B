/*
 * can.h
 *
 *  Created on: Nov 28, 2025
 *      Author: abrah
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include <stdint.h>
#include <stdbool.h>
#include "robot_types.h"
#include "main.h"

typedef struct {
    uint32_t id;           // CAN identifier (11-bit standard or 29-bit extended)
    uint8_t  data[64];     // Payload (supports CAN-FD up to 64 bytes)
    uint8_t  length;       // Actual data length in bytes
    uint32_t timestamp;    // When message was received (HAL_GetTick())
    bool     isExtended;   // true = 29-bit extended ID, false = 11-bit extended
} CANMessage;

static inline uint8_t dlc_to_bytes(uint32_t dlc) {
    static const uint8_t map[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    return (dlc < 16) ? map[dlc] : 0;
}
int drainAndUpdateCANMessages(void);
CANMessage* getCANMessageByID(uint32_t);

#endif /* INC_CAN_H_ */
