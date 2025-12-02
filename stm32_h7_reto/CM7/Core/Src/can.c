/*
 * can.c
 *
 *  Created on: Nov 28, 2025
 *      Author: abrah
 */


#include "can.h"
#include <string.h>
#include "main.h"

extern ControlMode currentMode;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_FilterTypeDef sFilterConfig;
extern FDCAN_TxHeaderTypeDef TxHeader;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern uint8_t TxData[8];
extern uint8_t RxData[8];
extern const uint8_t NUM_CAN_IDS;

// Storage for latest CAN messages by ID
extern CANMessage latestMsgs[];

// CAN FUNCTIONS
int drainAndUpdateCANMessages(void)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[64];
    HAL_StatusTypeDef status;
    int totalFramesRead = 0;

    // Drain entire FIFO, update matching IDs
    do {
        status = HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
        if (status == HAL_OK) {
            totalFramesRead++;

            // Check if this ID is in our tracking list
            for (uint8_t i = 0; i < NUM_CAN_IDS; i++) {
                bool idMatches = (RxHeader.Identifier == latestMsgs[i].id);
                bool typeMatches = ((RxHeader.IdType == FDCAN_STANDARD_ID) && !latestMsgs[i].isExtended) ||
                                   ((RxHeader.IdType == FDCAN_EXTENDED_ID) && latestMsgs[i].isExtended);

                if (idMatches && typeMatches) {
                    // Update this message slot
                    uint32_t raw = RxHeader.DataLength;
                    uint8_t len = (raw <= 64) ? (uint8_t)raw : dlc_to_bytes(raw >> 16);

                    latestMsgs[i].length = len;
                    latestMsgs[i].timestamp = HAL_GetTick();
                    memcpy(latestMsgs[i].data, RxData, len);
                    break;  // Found match, no need to check other slots
                }
            }
        }
    } while (status == HAL_OK);

    return totalFramesRead;
}

CANMessage* getCANMessageByID(uint32_t id)
{
    for (uint8_t i = 0; i < NUM_CAN_IDS; i++) {
        if (latestMsgs[i].id == id) {
            return &latestMsgs[i];
        }
    }
    return NULL;  // ID not tracked
}
