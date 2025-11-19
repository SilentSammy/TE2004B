/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "myprintf.h"
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>   // strtod
#include <errno.h>    // errno, ERANGE
#include <math.h>     // isfinite
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    uint32_t id;           // CAN identifier (11-bit standard or 29-bit extended)
    uint8_t  data[64];     // Payload (supports CAN-FD up to 64 bytes)
    uint8_t  length;       // Actual data length in bytes
    uint32_t timestamp;    // When message was received (HAL_GetTick())
    bool     isExtended;   // true = 29-bit extended ID, false = 11-bit extended
} CANMessage;

typedef struct {
    float roll;    // Roll angle in degrees (0.00° to 360.00°)
    float pitch;   // Pitch angle in degrees (0.00° to 360.00°)
    float yaw;     // Yaw angle in degrees (0.00° to 360.00°)
} Orientation;

typedef struct {
    float throttle;  // Throttle value (-1.0 to +1.0)
    float steering;  // Steering value (-1.0 to +1.0)
} MotorControl;

typedef struct {
    float gx;  // Angular velocity around X-axis (°/s)
    float gy;  // Angular velocity around Y-axis (°/s)
    float gz;  // Angular velocity around Z-axis (°/s)
} AngularVelocity;

typedef struct {
    float ax;  // Linear acceleration along X-axis (m/s²)
    float ay;  // Linear acceleration along Y-axis (m/s²)
    float az;  // Linear acceleration along Z-axis (m/s²)
} LinearAcceleration;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] = {0x10, 0x34, 0x54, 0x76, 0x98, 0x00, 0x11, 0x22};
uint8_t RxData[8];

// Storage for latest CAN messages by ID
CANMessage latestMsgs[] = {
    {.id = 0x123, .length = 0, .timestamp = 0, .isExtended = false},  // Encoder
    {.id = 0x124, .length = 0, .timestamp = 0, .isExtended = false},  // Orientation
    {.id = 0x125, .length = 0, .timestamp = 0, .isExtended = false},  // Motor Control
    {.id = 0x126, .length = 0, .timestamp = 0, .isExtended = false},  // Angular Velocity
    {.id = 0x127, .length = 0, .timestamp = 0, .isExtended = false}   // Linear Acceleration
};
#define NUM_CAN_IDS (sizeof(latestMsgs) / sizeof(latestMsgs[0]))
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// MOTORS AND SERVOS
void Turning_SetAngle(float steer)
{
    /* 0) Preprocess normalized input (-1..1) → (-90..90 degrees) */
    float angle = steer * 90.0f;

	/* 1) Clamp input */
    if (angle < -90.0f) angle = -90.0f;
    if (angle >  90.0f) angle =  90.0f;

    /* 2) Map to pulse width in microseconds */
    float pulseWidth = 1000.0f + (angle * 1000.0f) / 180.0f;   // 1000..2000 us

    /* 3) Convert to compare value and clamp to ARR */
    uint32_t arr   = __HAL_TIM_GET_AUTORELOAD(&htim13);        // e.g., 19999 for 20 ms frame
    int32_t  value = (int32_t)lroundf(pulseWidth);             // round to nearest tick

    if (value < 0) value = 0;
    if ((uint32_t)value > arr) value = (int32_t)arr;

    /* 4) Write compare register */
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, (uint32_t)value);
}
void SetEscSpeed(float value)
{
    /* 1) Clamp input range */
    if (value < -1.0f) value = -1.0f;
    if (value >  2.0f) value =  2.0f;

    /* 2) Adjust asymmetry:
          Forward (positive) -> halve output
          Reverse (negative) -> unchanged */
    if (value > 0.0f)
        value *= 0.5f;

    /* 3) Map to pulse width (µs)
          -1 → 1000 µs
           0 → 1500 µs
          +1 → 2000 µs
    */
    float pulseWidth = 1500.0f + (value * 500.0f);

    /* 4) Convert µs to timer ticks (1 tick = 1 µs assumed) */
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim14);
    int32_t ticks = (int32_t)lroundf(pulseWidth);

    /* 5) Clamp within timer range */
    if (ticks < 0) ticks = 0;
    if ((uint32_t)ticks > arr) ticks = (int32_t)arr;

    /* 6) Apply PWM */
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, (uint32_t)ticks);
}
void StopCarEsc(void)
{
    SetEscSpeed(0.0f);   // Neutral = 1500 µs pulse
}

// CAN FUNCTIONS
static inline uint8_t dlc_to_bytes(uint32_t dlc) {
    static const uint8_t map[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    return (dlc < 16) ? map[dlc] : 0;
}

/**
 * @brief Drain FIFO and update all tracked CAN messages in latestMsgs[]
 * @return Number of messages read from FIFO (total, not unique IDs)
 */
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

/**
 * @brief Get latest CAN message for a specific ID
 * @param id CAN identifier to retrieve
 * @return Pointer to CANMessage or NULL if not found
 */
CANMessage* getCANMessageByID(uint32_t id)
{
    for (uint8_t i = 0; i < NUM_CAN_IDS; i++) {
        if (latestMsgs[i].id == id) {
            return &latestMsgs[i];
        }
    }
    return NULL;  // ID not tracked
}

void loopPrintAllCAN(void)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[64];  // safe for FD
    const uint32_t heartbeatIntervalMs = 1000;
    uint32_t lastHeartbeat = HAL_GetTick();

    printf("\n\r[CAN Monitor] Listening for messages...\n\r");

    while (1) {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
            uint32_t raw = RxHeader.DataLength;
            uint8_t len  = (raw <= 64) ? (uint8_t)raw          // HAL variant where DataLength == bytes
                                       : dlc_to_bytes(raw >> 16); // HAL variant with DLC<<16

            printf("ID: 0x%03lX | Type: %s | Len: %u | Data:",
                   (unsigned long)RxHeader.Identifier,
                   (RxHeader.IdType == FDCAN_STANDARD_ID ? "STD" : "EXT"),
                   (unsigned)len);

            for (uint8_t i = 0; i < len; i++) printf(" %02X", RxData[i]);
            printf("\n\r");
        }

        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - lastHeartbeat) >= heartbeatIntervalMs) {
            printf("[CAN Monitor] Heartbeat... system alive (%lu ms)\n\r", (unsigned long)now);
            lastHeartbeat = now;
        }
        HAL_Delay(1);
    }
}

int32_t readEncoder(int32_t timeoutMs /* = -1 */)
{
    static int32_t lastPosition = 0;

    if (timeoutMs == -1)
        timeoutMs = 200;  // Default wait time

    int fifoCapacity = hfdcan1.Init.RxFifo0ElmtsNbr;

    // 1) Drain FIFO and update all tracked messages
    int framesRead = drainAndUpdateCANMessages();

    // 2) Get the encoder message
    CANMessage* encoderMsg = getCANMessageByID(0x123);
    if (encoderMsg != NULL && encoderMsg->length >= 4) {
        memcpy(&lastPosition, encoderMsg->data, sizeof(lastPosition));
    }

    // 3) If FIFO was full, wait for a fresh frame to ensure data isn't stale
    if (framesRead >= fifoCapacity && timeoutMs > 0) {
        uint32_t timeout = HAL_GetTick() + (uint32_t)timeoutMs;
        while (HAL_GetTick() < timeout) {
            int newFrames = drainAndUpdateCANMessages();
            if (newFrames > 0) {
                // Fresh data arrived, update encoder value
                encoderMsg = getCANMessageByID(0x123);
                if (encoderMsg != NULL && encoderMsg->length >= 4) {
                    memcpy(&lastPosition, encoderMsg->data, sizeof(lastPosition));
                }
                break;
            }
            HAL_Delay(1);
        }
    }

    return lastPosition;
}

bool readOrientation(Orientation* orientation, int32_t timeoutMs /* = -1 */)
{
    static Orientation lastOrientation = {0.0f, 0.0f, 0.0f};

    if (orientation == NULL) {
        return false;  // Invalid pointer
    }

    if (timeoutMs == -1)
        timeoutMs = 200;  // Default wait time

    int fifoCapacity = hfdcan1.Init.RxFifo0ElmtsNbr;

    // 1) Drain FIFO and update all tracked messages
    int framesRead = drainAndUpdateCANMessages();

    // 2) Get the orientation message
    CANMessage* orientMsg = getCANMessageByID(0x124);
    if (orientMsg != NULL && orientMsg->length >= 6) {
        // Decode roll (bytes 0-1), pitch (bytes 2-3), yaw (bytes 4-5)
        uint16_t rollRaw   = (uint16_t)orientMsg->data[0] | ((uint16_t)orientMsg->data[1] << 8);
        uint16_t pitchRaw  = (uint16_t)orientMsg->data[2] | ((uint16_t)orientMsg->data[3] << 8);
        uint16_t yawRaw    = (uint16_t)orientMsg->data[4] | ((uint16_t)orientMsg->data[5] << 8);

        // Convert to degrees (0.01° resolution)
        lastOrientation.roll  = rollRaw / 100.0f;
        lastOrientation.pitch = pitchRaw / 100.0f;
        lastOrientation.yaw   = yawRaw / 100.0f;
    }

    // 3) If FIFO was full, wait for a fresh frame to ensure data isn't stale
    if (framesRead >= fifoCapacity && timeoutMs > 0) {
        uint32_t timeout = HAL_GetTick() + (uint32_t)timeoutMs;
        while (HAL_GetTick() < timeout) {
            int newFrames = drainAndUpdateCANMessages();
            if (newFrames > 0) {
                // Fresh data arrived, update orientation
                orientMsg = getCANMessageByID(0x124);
                if (orientMsg != NULL && orientMsg->length >= 6) {
                    uint16_t rollRaw   = (uint16_t)orientMsg->data[0] | ((uint16_t)orientMsg->data[1] << 8);
                    uint16_t pitchRaw  = (uint16_t)orientMsg->data[2] | ((uint16_t)orientMsg->data[3] << 8);
                    uint16_t yawRaw    = (uint16_t)orientMsg->data[4] | ((uint16_t)orientMsg->data[5] << 8);

                    lastOrientation.roll  = rollRaw / 100.0f;
                    lastOrientation.pitch = pitchRaw / 100.0f;
                    lastOrientation.yaw   = yawRaw / 100.0f;
                }
                break;
            }
            HAL_Delay(1);
        }
    }

    // Copy to output
    *orientation = lastOrientation;

    // Return true if we have valid data (timestamp > 0 means we received at least one message)
    return (orientMsg != NULL && orientMsg->timestamp > 0);
}

bool readMotorControl(MotorControl* control, int32_t timeoutMs /* = -1 */)
{
    static MotorControl lastControl = {0.0f, 0.0f};

    if (control == NULL) {
        return false;  // Invalid pointer
    }

    if (timeoutMs == -1)
        timeoutMs = 200;  // Default wait time

    int fifoCapacity = hfdcan1.Init.RxFifo0ElmtsNbr;

    // 1) Drain FIFO and update all tracked messages
    int framesRead = drainAndUpdateCANMessages();

    // 2) Get the motor control message
    CANMessage* controlMsg = getCANMessageByID(0x125);
    if (controlMsg != NULL && controlMsg->length >= 4) {
        // Decode throttle (bytes 0-1), steering (bytes 2-3)
        int16_t throttleRaw = (int16_t)((uint16_t)controlMsg->data[0] | ((uint16_t)controlMsg->data[1] << 8));
        int16_t steeringRaw = (int16_t)((uint16_t)controlMsg->data[2] | ((uint16_t)controlMsg->data[3] << 8));

        // Convert to normalized float (-1.0 to +1.0)
        lastControl.throttle = throttleRaw / 1000.0f;
        lastControl.throttle *= 2;
        lastControl.steering = steeringRaw / 1000.0f;
    }

    // 3) If FIFO was full, wait for a fresh frame to ensure data isn't stale
    if (framesRead >= fifoCapacity && timeoutMs > 0) {
        uint32_t timeout = HAL_GetTick() + (uint32_t)timeoutMs;
        while (HAL_GetTick() < timeout) {
            int newFrames = drainAndUpdateCANMessages();
            if (newFrames > 0) {
                // Fresh data arrived, update motor control
                controlMsg = getCANMessageByID(0x125);
                if (controlMsg != NULL && controlMsg->length >= 4) {
                    int16_t throttleRaw = (int16_t)((uint16_t)controlMsg->data[0] | ((uint16_t)controlMsg->data[1] << 8));
                    int16_t steeringRaw = (int16_t)((uint16_t)controlMsg->data[2] | ((uint16_t)controlMsg->data[3] << 8));

                    lastControl.throttle = throttleRaw / 1000.0f;
                    lastControl.steering = steeringRaw / 1000.0f;
                }
                break;
            }
            HAL_Delay(1);
        }
    }

    // Copy to output
    *control = lastControl;

    // Return true if we have valid data (timestamp > 0 means we received at least one message)
    return (controlMsg != NULL && controlMsg->timestamp > 0);
}

bool readAngularVelocity(AngularVelocity* angvel, int32_t timeoutMs /* = -1 */)
{
    static AngularVelocity lastAngvel = {0.0f, 0.0f, 0.0f};

    if (angvel == NULL) {
        return false;
    }

    if (timeoutMs == -1)
        timeoutMs = 200;

    int fifoCapacity = hfdcan1.Init.RxFifo0ElmtsNbr;
    int framesRead = drainAndUpdateCANMessages();

    CANMessage* angvelMsg = getCANMessageByID(0x126);
    if (angvelMsg != NULL && angvelMsg->length >= 6) {
        int16_t gx_raw = (int16_t)((uint16_t)angvelMsg->data[0] | ((uint16_t)angvelMsg->data[1] << 8));
        int16_t gy_raw = (int16_t)((uint16_t)angvelMsg->data[2] | ((uint16_t)angvelMsg->data[3] << 8));
        int16_t gz_raw = (int16_t)((uint16_t)angvelMsg->data[4] | ((uint16_t)angvelMsg->data[5] << 8));

        lastAngvel.gx = gx_raw / 10.0f;
        lastAngvel.gy = gy_raw / 10.0f;
        lastAngvel.gz = gz_raw / 10.0f;
    }

    if (framesRead >= fifoCapacity && timeoutMs > 0) {
        uint32_t timeout = HAL_GetTick() + (uint32_t)timeoutMs;
        while (HAL_GetTick() < timeout) {
            int newFrames = drainAndUpdateCANMessages();
            if (newFrames > 0) {
                angvelMsg = getCANMessageByID(0x126);
                if (angvelMsg != NULL && angvelMsg->length >= 6) {
                    int16_t gx_raw = (int16_t)((uint16_t)angvelMsg->data[0] | ((uint16_t)angvelMsg->data[1] << 8));
                    int16_t gy_raw = (int16_t)((uint16_t)angvelMsg->data[2] | ((uint16_t)angvelMsg->data[3] << 8));
                    int16_t gz_raw = (int16_t)((uint16_t)angvelMsg->data[4] | ((uint16_t)angvelMsg->data[5] << 8));

                    lastAngvel.gx = gx_raw / 10.0f;
                    lastAngvel.gy = gy_raw / 10.0f;
                    lastAngvel.gz = gz_raw / 10.0f;
                }
                break;
            }
            HAL_Delay(1);
        }
    }

    *angvel = lastAngvel;
    return (angvelMsg != NULL && angvelMsg->timestamp > 0);
}

bool readLinearAcceleration(LinearAcceleration* accel, int32_t timeoutMs /* = -1 */)
{
    static LinearAcceleration lastAccel = {0.0f, 0.0f, 0.0f};

    if (accel == NULL) {
        return false;
    }

    if (timeoutMs == -1)
        timeoutMs = 200;

    int fifoCapacity = hfdcan1.Init.RxFifo0ElmtsNbr;
    int framesRead = drainAndUpdateCANMessages();

    CANMessage* accelMsg = getCANMessageByID(0x127);
    if (accelMsg != NULL && accelMsg->length >= 6) {
        int16_t ax_raw = (int16_t)((uint16_t)accelMsg->data[0] | ((uint16_t)accelMsg->data[1] << 8));
        int16_t ay_raw = (int16_t)((uint16_t)accelMsg->data[2] | ((uint16_t)accelMsg->data[3] << 8));
        int16_t az_raw = (int16_t)((uint16_t)accelMsg->data[4] | ((uint16_t)accelMsg->data[5] << 8));

        lastAccel.ax = ax_raw / 100.0f;
        lastAccel.ay = ay_raw / 100.0f;
        lastAccel.az = az_raw / 100.0f;
    }

    if (framesRead >= fifoCapacity && timeoutMs > 0) {
        uint32_t timeout = HAL_GetTick() + (uint32_t)timeoutMs;
        while (HAL_GetTick() < timeout) {
            int newFrames = drainAndUpdateCANMessages();
            if (newFrames > 0) {
                accelMsg = getCANMessageByID(0x127);
                if (accelMsg != NULL && accelMsg->length >= 6) {
                    int16_t ax_raw = (int16_t)((uint16_t)accelMsg->data[0] | ((uint16_t)accelMsg->data[1] << 8));
                    int16_t ay_raw = (int16_t)((uint16_t)accelMsg->data[2] | ((uint16_t)accelMsg->data[3] << 8));
                    int16_t az_raw = (int16_t)((uint16_t)accelMsg->data[4] | ((uint16_t)accelMsg->data[5] << 8));

                    lastAccel.ax = ax_raw / 100.0f;
                    lastAccel.ay = ay_raw / 100.0f;
                    lastAccel.az = az_raw / 100.0f;
                }
                break;
            }
            HAL_Delay(1);
        }
    }

    *accel = lastAccel;
    return (accelMsg != NULL && accelMsg->timestamp > 0);
}

void loopPrintEncoder(void)
{
    const uint32_t printIntervalMs = 100;   // print every 100 ms
    const uint32_t heartbeatIntervalMs = 1000; // heartbeat every 1 s
    uint32_t lastPrint = HAL_GetTick();
    uint32_t lastHeartbeat = HAL_GetTick();

    printf("\n\r[Encoder Monitor] Starting...\n\r");

    while (1)
    {
        uint32_t now = HAL_GetTick();

        // 1) Periodically read encoder value
        if (now - lastPrint >= printIntervalMs)
        {
            int32_t encoderCount = readEncoder(-1);
            printf("Encoder: %ld\n\r", (long)encoderCount);
            lastPrint = now;
        }

        // 2) Heartbeat every second
        if (now - lastHeartbeat >= heartbeatIntervalMs)
        {
            printf("[Encoder Monitor] Alive (%lu ms)\n\r", (unsigned long)now);
            lastHeartbeat = now;
        }

        HAL_Delay(1);
    }
}

void loopPrintOrientation(void) {
    const uint32_t printIntervalMs = 100;      // print every 100 ms
    const uint32_t heartbeatIntervalMs = 1000; // heartbeat every 1 s
    uint32_t lastPrint = HAL_GetTick();
    uint32_t lastHeartbeat = HAL_GetTick();

    printf("\n\r[Orientation Monitor] Starting...\n\r");

    while (1)
    {
        uint32_t now = HAL_GetTick();

        // 1) Periodically read orientation values
        if (now - lastPrint >= printIntervalMs)
        {
            Orientation orient;
            if (readOrientation(&orient, -1)) {
                printf("Roll: %6.2f° | Pitch: %6.2f° | Yaw: %6.2f°\n\r", orient.roll, orient.pitch, orient.yaw);
            } else {
                printf("Orientation: [No data]\n\r");
            }
            lastPrint = now;
        }

        // 2) Heartbeat every second
        if (now - lastHeartbeat >= heartbeatIntervalMs)
        {
            printf("[Orientation Monitor] Alive (%lu ms)\n\r", (unsigned long)now);
            lastHeartbeat = now;
        }

        HAL_Delay(1);
    }
}

void loopPrintMotorControl(void) {
    const uint32_t printIntervalMs = 100;      // print every 100 ms
    const uint32_t heartbeatIntervalMs = 1000; // heartbeat every 1 s
    uint32_t lastPrint = HAL_GetTick();
    uint32_t lastHeartbeat = HAL_GetTick();

    printf("\n\r[Motor Control Monitor] Starting...\n\r");

    while (1)
    {
        uint32_t now = HAL_GetTick();

        // 1) Read motor control values and apply to hardware
        MotorControl control;
        if (readMotorControl(&control, 0)) {  // Non-blocking read
            // Apply steering to servo
            Turning_SetAngle(control.steering);
            
            // Apply throttle to ESC
            SetEscSpeed(control.throttle);
        }

        // 2) Periodically print motor control values
        if (now - lastPrint >= printIntervalMs)
        {
            if (readMotorControl(&control, 0)) {
                printf("Throttle: %+5.2f | Steering: %+5.2f\n\r", control.throttle, control.steering);
            } else {
                printf("Motor Control: [No data]\n\r");
            }
            lastPrint = now;
        }

        // 3) Heartbeat every second
        if (now - lastHeartbeat >= heartbeatIntervalMs)
        {
            printf("[Motor Control Monitor] Alive (%lu ms)\n\r", (unsigned long)now);
            lastHeartbeat = now;
        }

        HAL_Delay(1);
    }
}

void loopPrintAll(void) {
    const uint32_t printIntervalMs = 100;
    const uint32_t heartbeatIntervalMs = 1000;
    uint32_t lastPrint = HAL_GetTick();
    uint32_t lastHeartbeat = HAL_GetTick();

    printf("\n\r[Combined Monitor] Starting...\n\r");

    while (1)
    {
        uint32_t now = HAL_GetTick();

        // 1) Apply motor control commands
        MotorControl control;
        if (readMotorControl(&control, 0)) {
            Turning_SetAngle(control.steering);
            SetEscSpeed(control.throttle);
        }

        // 2) Periodically print all values
        if (now - lastPrint >= printIntervalMs)
        {
            drainAndUpdateCANMessages();

            int32_t encoderCount = readEncoder(0);
            Orientation orient;
            AngularVelocity angvel;
            LinearAcceleration accel;
            bool hasOrientation = readOrientation(&orient, 0);
            bool hasAngvel = readAngularVelocity(&angvel, 0);
            bool hasAccel = readLinearAcceleration(&accel, 0);
            bool hasControl = readMotorControl(&control, 0);

            printf("Enc:%8ld | ", (long)encoderCount);
            
            if (hasOrientation) {
                printf("R:%6.2f P:%6.2f Y:%6.2f | ", orient.roll, orient.pitch, orient.yaw);
            } else {
                printf("Orient:[No data] | ");
            }

            if (hasAngvel) {
                printf("Gx:%+6.1f Gy:%+6.1f Gz:%+6.1f | ", angvel.gx, angvel.gy, angvel.gz);
            } else {
                printf("Angvel:[No data] | ");
            }

            if (hasAccel) {
                printf("Ax:%+5.2f Ay:%+5.2f Az:%+5.2f | ", accel.ax, accel.ay, accel.az);
            } else {
                printf("Accel:[No data] | ");
            }

            if (hasControl) {
                printf("T:%+5.2f S:%+5.2f\n\r", control.throttle, control.steering);
            } else {
                printf("Ctrl:[No data]\n\r");
            }

            lastPrint = now;
        }

        // 3) Heartbeat every second
        if (now - lastHeartbeat >= heartbeatIntervalMs)
        {
            printf("[Combined Monitor] Alive (%lu ms)\n\r", (unsigned long)now);
            lastHeartbeat = now;
        }

        HAL_Delay(1);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
//  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_FDCAN1_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
  Turning_SetAngle(0);
//  loopPrintAllCAN();
 loopPrintAll();
//  loopPrintEncoder();
//  loopPrintOrientation();
  // loopPrintMotorControl();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 2;
  hfdcan1.Init.NominalSyncJumpWidth = 8;
  hfdcan1.Init.NominalTimeSeg1 = 0x1F;
  hfdcan1.Init.NominalTimeSeg2 = 8;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 50;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  /*AAO+*/
          sFilterConfig.IdType = FDCAN_STANDARD_ID;
          sFilterConfig.FilterIndex = 0;
          sFilterConfig.FilterType = FDCAN_FILTER_MASK;
          sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
          sFilterConfig.FilterID1 = 0x000;   // base ID (don't care)
          sFilterConfig.FilterID2 = 0x000;   // mask = 0 → accept all messages

          /* Configure global filter to reject all non-matching frames */
          HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                       FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

          if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
            {
               /* Filter configuration Error */
               Error_Handler();
            }
           /* Start the FDCAN module */
          if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
            }
               /* Start Error */
          if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
            }
               /* Notification Error */

           /* Configure Tx buffer message */
          TxHeader.Identifier = 0x111;
          TxHeader.IdType = FDCAN_STANDARD_ID;
          TxHeader.TxFrameType = FDCAN_DATA_FRAME;
          TxHeader.DataLength = FDCAN_DLC_BYTES_12;
          TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
          TxHeader.BitRateSwitch = FDCAN_BRS_ON;
          TxHeader.FDFormat = FDCAN_FD_CAN;
          TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
          TxHeader.MessageMarker = 0x00;
         /*AAO-*/
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 79;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 19999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 79;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 19999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
