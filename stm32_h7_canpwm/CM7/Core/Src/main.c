/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define UART_MAILBOX_IMPL     // <-- defines the implementation once
#include "uart_mailbox.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
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
float turn = 0.5f;
int8_t throttle = 0;
int8_t steering = 0;
float speed = 1.0f;   // throttle multiplier

FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] = {0x10, 0x34, 0x54, 0x76, 0x98, 0x00, 0x11, 0x22};
uint8_t RxData[8];
static uart_mb_t *g_mb = NULL;   // <-- app-owned “global”
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
void handleToggle(char **args, int argc);
void handleSetTurn(char **args, int argc);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// COMMANDS
#ifndef CMD_BUF_SZ
#define CMD_BUF_SZ 128
#endif
#define MAX_ARGS 8
typedef void (*CommandHandler)(char **args, int argc);
typedef struct {
    const char *name;
    CommandHandler handler;
} CommandEntry;
typedef struct {
    CommandHandler handler;
    char *args[MAX_ARGS];
    int argc;
    bool valid;
} ParsedCommand;
const CommandEntry commandTable[] = {
    { "TOGGLE", handleToggle },
    { "SETTURN", handleSetTurn },
};
#define NUM_COMMANDS (sizeof(commandTable) / sizeof(commandTable[0]))

// CAN FUNCTIONS
static inline uint8_t dlc_to_bytes(uint32_t dlc) {
    static const uint8_t map[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    return (dlc < 16) ? map[dlc] : 0;
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
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    HAL_StatusTypeDef status;

    if (timeoutMs == -1)
        timeoutMs = 200;  // Default wait time

    // printf("\n\r[readEncoder] Draining FIFO...\n\r");

    int framesRead = 0;
    int fifoCapacity = hfdcan1.Init.RxFifo0ElmtsNbr;

    // 1) Drain FIFO0 completely, keep the latest valid frame
    do {
        status = HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
        if (status == HAL_OK &&
            RxHeader.IdType == FDCAN_STANDARD_ID &&
            RxHeader.Identifier == 0x123)
        {
            int32_t val;
            memcpy(&val, RxData, sizeof(val));
            lastPosition = val;
            framesRead++;
        }
    } while (status == HAL_OK);

    // printf("[readEncoder] Frames drained: %d | FIFO capacity: %d | Latest value: %ld\n\r",
    //        framesRead, fifoCapacity, (long)lastPosition);

    // 2) If FIFO was full, decide whether to wait for a fresh frame
    if (framesRead >= fifoCapacity && timeoutMs > 0) {
        // printf("[readEncoder] FIFO likely full -> waiting up to %ld ms for a fresh frame...\n\r",
        //        (long)timeoutMs);

        uint32_t timeout = HAL_GetTick() + (uint32_t)timeoutMs;
        uint32_t waited = 0;
        while (HAL_GetTick() < timeout) {
            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK &&
                RxHeader.IdType == FDCAN_STANDARD_ID &&
                RxHeader.Identifier == 0x123)
            {
                int32_t val;
                memcpy(&val, RxData, sizeof(val));
                lastPosition = val;
                // printf("[readEncoder] New frame received after %lu ms | Latest value: %ld\n\r",
                //        (unsigned long)waited, (long)lastPosition);
                break;
            }
            HAL_Delay(1);
            waited++;
        }

        // if (HAL_GetTick() >= timeout)
        //     printf("[readEncoder] Timeout (%ld ms) reached, returning last known value: %ld\n\r",
        //            (long)timeoutMs, (long)lastPosition);
    }
    // else if (timeoutMs == 0) {
    //     printf("[readEncoder] Non-blocking mode, returning immediately (may be stale): %ld\n\r",
    //            (long)lastPosition);
    // }
    // else {
    //     printf("[readEncoder] FIFO not full, returning immediately: %ld\n\r",
    //            (long)lastPosition);
    // }

    return lastPosition;
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

    char msg[64];
    int len = snprintf(msg, sizeof(msg), "Stering set to:%.2f\r\n", steer);
    uart_mb_send_all((uint8_t*)msg, len, HAL_MAX_DELAY);
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
void debugSetup() {
	float dbg_spd = 0.5f;
	float dbg_str = 0.0f;

	while (1) {
		int32_t current = readEncoder(-1);
		SetEscSpeed(dbg_spd);
		Turning_SetAngle(dbg_str);
		StopCarEsc();
	}
}

// NAVIGATION
void goDistance(float meters, float speed)
{
    const float COUNTS_PER_METER = 16066.0f;
    int32_t targetCounts = (int32_t)(meters * COUNTS_PER_METER);

    printf("\n\r=== Move Forward %.2f m ===\n\r", meters);
    printf("Target counts: %ld\n\r", (long)targetCounts);

    // Make sure car is stopped
    SetEscSpeed(0);
    HAL_Delay(1000);

    int32_t startCount = readEncoder(-1);
    int32_t current = startCount;
    int32_t delta = 0;


    uint32_t lastPrint = HAL_GetTick();  // track print intervals

    printf("Moving forward at %.2f...\n\r", speed);
    while (1)
    {
		SetEscSpeed(speed);
        current = readEncoder(-1);
        delta = current - startCount;

        // Stop once target reached
        if (delta >= targetCounts)
        {
            StopCarEsc();
            printf("\n\rTarget reached! Δ=%ld\n\r", (long)delta);
            break;
        }

        // Print every 200 ms (not every loop)
        if (HAL_GetTick() - lastPrint >= 200)
        {
            printf("Δ: %ld / %ld\r", (long)delta, (long)targetCounts);
            lastPrint = HAL_GetTick();
        }

        // Short loop delay (fast polling, low CPU use)
        HAL_Delay(2);
    }

    int32_t finalCount = readEncoder(-1);
    printf("Final count: %ld | Total Δ: %ld\n\r",
           (long)finalCount, (long)(finalCount - startCount));
}

void goDistanceP(float meters, float kP, float maxSpeed)
{
    const float COUNTS_PER_METER = 16066.0f;
    const int32_t DEADBAND = 100;     // counts
    const float MIN_THROTTLE = 0.08f; // minimum throttle magnitude

    printf("\n\r=== Move %.2f m (kP=%.3f, maxSpeed=%.2f) ===\n\r",
           meters, kP, maxSpeed);

    // Ensure car is stopped and ESC armed
    printf("Ensuring vehicle is stopped...\n\r");
    SetEscSpeed(0.0f);
    HAL_Delay(1000);

    int32_t startCount = readEncoder(-1);
    uint32_t lastPrint = HAL_GetTick();

    printf("Starting proportional distance hold...\n\r");

    while (1)
    {
        // Recalculate target dynamically (for debugger tuning)
        int32_t targetCounts = (int32_t)(meters * COUNTS_PER_METER);

        int32_t current = readEncoder(-1);
        int32_t delta   = current - startCount;
        int32_t error   = targetCounts - delta;

        float speedCmd = 0.0f;

        // Apply proportional control only outside deadband
        if (error > DEADBAND)
            speedCmd =  kP * error;
        else if (error < -DEADBAND)
            speedCmd =  kP * error;
        else
            speedCmd = 0.0f;

        // Clamp to max speed
        if (speedCmd >  maxSpeed) speedCmd =  maxSpeed;
        if (speedCmd < -maxSpeed) speedCmd = -maxSpeed;

        // Enforce minimum throttle (only if nonzero command)
        if (speedCmd > 0 && speedCmd <  MIN_THROTTLE) speedCmd =  MIN_THROTTLE;
        if (speedCmd < 0 && speedCmd > -MIN_THROTTLE) speedCmd = -MIN_THROTTLE;

        SetEscSpeed(speedCmd);

        // Periodic debug print
        if (HAL_GetTick() - lastPrint >= 250)
        {
            printf("Δ=%ld | err=%ld | cmd=%.3f\n\r",
                   (long)delta, (long)error, speedCmd);
            lastPrint = HAL_GetTick();
        }

        // No delay — tight control loop
    }
}

float pidDistance(float error, float Kp, float Ki, float Kd, float outMin, float outMax)
{
    static float integral = 0.0f;
    static float prevError = 0.0f;
    static uint32_t lastTick = 0;

    uint32_t now = HAL_GetTick();
    float dt = 0.0f;

    if (lastTick == 0)
    {
        // first call
        lastTick = now;
        prevError = error;
        return 0.0f;
    }

    dt = (now - lastTick) / 1000.0f; // ms → seconds
    lastTick = now;

    // Protect against absurd dt (e.g., after pause)
    if (dt <= 0.0f || dt > 1.0f)
        dt = 0.01f;

    // --- PID calculation ---
    integral += error * dt;
    float derivative = (error - prevError) / dt;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Clamp output
    if (output >  outMax) output =  outMax;
    if (output < outMin) output = outMin;

    prevError = error;
    return output;
}
void goDistancePID_live(void)
{
    // --- CONFIGURABLE PARAMETERS (editable live in debugger) ---
    UART_HandleTypeDef *uart = &huart3;  // switch UART here

    float Kp = 0.001f;   // proportional gain
    float Ki = 0.0f;   // integral gain
    float Kd = 0.0f;   // derivative gain
    float maxSpeed = 0.5f; // output saturation (ESC command)
    float minThrottle = 0.0f;
    int32_t deadband = 100;
    const float COUNTS_PER_METER = 16066.0f;

    // --- INTERNAL STATE ---
    float targetMeters = 0.0f;
    char rxBuf[32];
    int rxIndex = 0;
    char msg[128];
    int len;

    const uint32_t PRINT_INTERVAL_MS = 250;
    uint32_t lastPrint = HAL_GetTick();
    int printEnabled = 1;

    SetEscSpeed(0.0f);
    HAL_Delay(1000);

    len = snprintf(msg, sizeof(msg),
                   "\n\r=== Live PID Distance Control ===\n\r"
                   "Type a distance (m) + ENTER to update target.\n\r");
    HAL_UART_Transmit(uart, (uint8_t*)msg, len, HAL_MAX_DELAY);

    int32_t startCount = readEncoder(-1);

    while (1)
    {
        // --- UART input handling ---
        uint8_t ch;
        if (HAL_UART_Receive(uart, &ch, 1, 0) == HAL_OK)
        {
            // Echo typed character
            HAL_UART_Transmit(uart, &ch, 1, HAL_MAX_DELAY);

            if (ch == '\r' || ch == '\n')
            {
                rxBuf[rxIndex] = '\0';
                if (rxIndex > 0)
                {
                    float newTarget;
                    if (sscanf(rxBuf, "%f", &newTarget) == 1)
                    {
                        targetMeters = newTarget;
                        len = snprintf(msg, sizeof(msg),
                                       "\n\r[Target: %.3f m]\n\r", targetMeters);
                        HAL_UART_Transmit(uart, (uint8_t*)msg, len, HAL_MAX_DELAY);
                    }
                    else
                    {
                        const char invalidMsg[] = "\n\r[Invalid input]\n\r";
                        HAL_UART_Transmit(uart, (uint8_t*)invalidMsg,
                                          strlen(invalidMsg), HAL_MAX_DELAY);
                    }
                    rxIndex = 0;
                }
            }
            else if (rxIndex < (int)sizeof(rxBuf) - 1)
            {
                rxBuf[rxIndex++] = ch;
            }
        }

        // --- Encoder + error calculation ---
        int32_t targetCounts = (int32_t)(targetMeters * COUNTS_PER_METER);
        int32_t current = readEncoder(-1);
        int32_t delta   = current - startCount;
        int32_t error   = targetCounts - delta;

        // --- PID control output ---
        float speedCmd = 0.0f;

        if (error >  deadband || error < -deadband)
            speedCmd = pidDistance((float)error, Kp, Ki, Kd, -maxSpeed, maxSpeed);
        else
            speedCmd = 0.0f;

        // --- Minimum throttle enforcement ---
        if (speedCmd > 0 && speedCmd <  minThrottle) speedCmd =  minThrottle;
        if (speedCmd < 0 && speedCmd > -minThrottle) speedCmd = -minThrottle;

        SetEscSpeed(speedCmd);

        // --- Smarter print logic ---
        if (HAL_GetTick() - lastPrint >= PRINT_INTERVAL_MS)
        {
            lastPrint = HAL_GetTick();

            if (speedCmd != 0.0f || error > deadband || error < -deadband)
            {
                printEnabled = 1;
                len = snprintf(msg, sizeof(msg),
                               "Δ=%ld e=%ld u=%.2f\r\n",
                               (long)delta, (long)error, speedCmd);
                HAL_UART_Transmit(uart, (uint8_t*)msg, len, HAL_MAX_DELAY);
            }
            else if (printEnabled)
            {
                const char doneMsg[] = "[At target]\r\n";
                HAL_UART_Transmit(uart, (uint8_t*)doneMsg, strlen(doneMsg), HAL_MAX_DELAY);
                printEnabled = 0;
            }
        }
    }
}

// SERIAL COMMS
ParsedCommand parseCommandLine(char *input, const CommandEntry *table, int tableSize)
{
    ParsedCommand result = {0};
    result.valid = false;

    if (!input || *input == '\0')
        return result;

    // Trim leading spaces
    while (isspace((unsigned char)*input)) input++;
    if (*input == '\0')
        return result;

    // Find the function name / argument separator
    char *colon = strchr(input, ':');
    char *funcEnd = colon ? colon : input + strlen(input);

    // Extract command name
    *funcEnd = '\0';
    for (char *p = input; *p; ++p)
        *p = toupper((unsigned char)*p);

    // Look up the handler
    for (int i = 0; i < tableSize; i++) {
        if (strcmp(table[i].name, input) == 0) {
            result.handler = table[i].handler;
            result.valid = true;
            break;
        }
    }

    if (!result.valid)
        return result;  // unknown command

    // No colon? → no arguments
    if (!colon)
        return result;

    // Parse arguments after the colon
    char *argString = colon + 1;
    int argc = 0;

    while (argc < MAX_ARGS) {
        // Find next comma or end
        char *comma = strchr(argString, ',');
        if (comma)
            *comma = '\0';

        // Trim spaces
        while (isspace((unsigned char)*argString)) argString++;
        if (*argString)
            result.args[argc++] = argString;

        if (!comma)
            break;  // no more commas
        argString = comma + 1;
    }

    result.argc = argc;
    return result;
}
void decodeDirection(char input, int8_t *throttle, int8_t *steering)
{
    switch (input) {
        case 'F': *throttle =  1; *steering =  0; break;  // Forward
        case 'B': *throttle = -1; *steering =  0; break;  // Backward
        case 'L': *throttle =  0; *steering = -1; break;  // Left
        case 'R': *throttle =  0; *steering =  1; break;  // Right
        case 'G': *throttle =  1; *steering = -1; break;  // Forward + Left
        case 'H': *throttle =  1; *steering =  1; break;  // Forward + Right
        case 'I': *throttle = -1; *steering = -1; break;  // Backward + Left
        case 'J': *throttle = -1; *steering =  1; break;  // Backward + Right
        case 'S': *throttle =  0; *steering =  0; break;  // Stop
        default: break;
    }
}
void decodeSpeedChar(char input, float *speed)
{
    if (input >= '0' && input <= '9') {
        int digit = input - '0';
        *speed = (digit + 1) / 10.0f;   // '0'–'9' -> 0.1–1.0
    }
}
void echo() {
	uart_mb_t *mb = g_mb;
    const char *hello = "Mailbox echo ready\r\n";
    HAL_UART_Transmit(mb->huart, (uint8_t*)hello, strlen(hello), HAL_MAX_DELAY);

    uint8_t b;
    for (;;) {
        while (uart_mb_get(mb, &b)) {
            HAL_UART_Transmit(mb->huart, &b, 1, HAL_MAX_DELAY);
        }
        HAL_Delay(2);
    }
}
void echo_all(void) {
    uint8_t b;
    for (;;) {
        while (uart_mb_get_any(&b)) {
            uart_mb_send_all(&b, 1, 1000);
        }
        HAL_Delay(2);
    }
}
static void processShortCmd(uint8_t cmd)
{
    char msg[64];

    decodeDirection(cmd, &throttle, &steering);
    decodeSpeedChar(cmd, &speed);

    // Scale throttle to ±2.0, steering fixed to ±0.5
    float appliedThrottle = throttle * speed * 2.0f;
    float appliedSteering = steering * turn;

    SetEscSpeed(appliedThrottle);
    Turning_SetAngle(appliedSteering);

    int len = snprintf(msg, sizeof(msg),
                       "Cmd:%c | Th:%.2f | St:%.2f | Spd:%.1f\r\n",
                       cmd, appliedThrottle, appliedSteering, speed);

    uart_mb_send_all((uint8_t*)msg, len, HAL_MAX_DELAY);
}
static void processLongCmd(const uint8_t *buf, size_t len)
{
    // 1) Make a mutable, NUL-terminated copy for parseCommandLine()
    char line[CMD_BUF_SZ + 1];
    if (len > CMD_BUF_SZ) len = CMD_BUF_SZ;      // clamp
    memcpy(line, buf, len);
    line[len] = '\0';

    // 2) Parse: NAME[:arg1,arg2,...]
    ParsedCommand pc = parseCommandLine(line, commandTable, (int)NUM_COMMANDS);

    // 3) Dispatch or report error
    if (pc.valid && pc.handler) {
        pc.handler(pc.args, pc.argc);
    } else {
        char msg[64];
        int n = snprintf(msg, sizeof(msg), "ERR:UNKNOWN_CMD\r\n");
        uart_mb_send_all((uint8_t*)msg, (uint16_t)n, HAL_MAX_DELAY);
    }
}
void commandLoop(void)
{
    static uint8_t  collecting = 0;
    static uint16_t cmdLen = 0;
    static uint8_t  cmdBuf[CMD_BUF_SZ];

    uint8_t b;

    for (;;)
    {
        while (uart_mb_get_any(&b))
        {
            if (!collecting) {
                if (b == '#') {
                    collecting = 1;
                    cmdLen = 0;
                } else if (b != ';') {
                    processShortCmd(b);
                }
            } else {
                if (b == ';') {
                    if (cmdLen > 0) processLongCmd(cmdBuf, cmdLen);
                    collecting = 0;
                    cmdLen = 0;
                } else {
                    if (cmdLen < CMD_BUF_SZ) {
                        cmdBuf[cmdLen++] = b;
                    } else {
                        // Overflow policy: flush current chunk then continue
                        processLongCmd(cmdBuf, cmdLen);
                        cmdLen = 0;
                        cmdBuf[cmdLen++] = b;
                    }
                }
            }
        }
        HAL_Delay(2);
    }
}

// COMMAND HANDLERS
void handleToggle(char **args, int argc)
{
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
}
static inline char *rtrim(char *s) {
    size_t n = strlen(s);
    while (n && (unsigned char)s[n-1] <= ' ') s[--n] = '\0';  // trims space, \r, \n, \t
    return s;
}
void handleSetTurn(char **args, int argc)
{
    if (argc < 1) {
        uart_mb_send_all_cstr("ERR:Missing argument\r\n", 1000);
        return;
    }

    char tmp[32];
    size_t L = strlen(args[0]);
    if (L >= sizeof(tmp)) L = sizeof(tmp) - 1;
    memcpy(tmp, args[0], L);
    tmp[L] = '\0';

    rtrim(tmp);
    for (char *p = tmp; *p; ++p) if (*p == ',') *p = '.';  // allow "0,2"

    errno = 0;
    char *end = NULL;
    double dv = strtod(tmp, &end);  // <— portable
    while (end && *end && (unsigned char)*end <= ' ') end++;  // ignore trailing spaces

    if (end == tmp || (end && *end != '\0') || !isfinite(dv) || errno == ERANGE) {
        uart_mb_send_all_cstr("ERR:Invalid float\r\n", 1000);
        return;
    }

    turn = (float)dv;
    float appliedSteering = steering * turn;
    Turning_SetAngle(appliedSteering);

    char msg[48];
    snprintf(msg, sizeof(msg), "OK:TURN=%.3f\r\n", (double)turn);
    uart_mb_send_all_cstr(msg, 1000);
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
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
//  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
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
  uart_mb_register(&huart3, &g_mb);  // choose which UART feeds the global mailbox
  uart_mb_register(&huart2, NULL);
//  echo_all();
//  commandLoop();
  loopPrintAllCAN();
//  loopPrintEncoder();
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
