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
#include "cmsis_os.h"
#include "robot_types.h"
#include "can.h"
#include "sensors.h"
#include "actuators.h"

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

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
ControlMode currentMode = MODE_MANUAL;  // Default to manual
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
	{.id = 0x127, .length = 0, .timestamp = 0, .isExtended = false},   // Linear Acceleration
    {.id = 0x128, .length = 0, .timestamp = 0, .isExtended = false},   // Pseudo-GPS
    {.id = 0x129, .length = 0, .timestamp = 0, .isExtended = false}   // Waypoint2D
};
const uint8_t NUM_CAN_IDS = sizeof(latestMsgs) / sizeof(latestMsgs[0]);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void debugLoop() {
	float dbg_spd = 0.5f;
	float dbg_str = 0.0f;

	while (1) {
		SetEscSpeed(dbg_spd);
		Turning_SetAngle(dbg_str);
		StopCarEsc();
	}
}

// HELPERS
float unwrapAngle(float wrappedAngle) {
    static float unwrappedValue = 0.0f;
    static bool initialized = false;
    
    if (!initialized) {
        // First call - initialize with current angle
        unwrappedValue = wrappedAngle;
        initialized = true;
        return unwrappedValue;
    }
    
    // Calculate previous wrapped equivalent
    float wrappedPrev = fmodf(unwrappedValue, 360.0f);
    if (wrappedPrev < 0.0f) wrappedPrev += 360.0f;
    
    // Calculate delta with current reading
    float delta = wrappedAngle - wrappedPrev;
    
    // Normalize delta to [-180, 180]
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    
    // Update unwrapped value
    unwrappedValue += delta;
    
    return unwrappedValue;
}

void differentialToAckermann(float omega, bool reset, float* throttle, float* steering) {
    static int32_t baseEncoderPosition = 0;
    static bool movingForward = true;  // Current movement direction
    const int32_t danceAmplitude = 500;  // How far to move from baseline
    const float throttleStrength = 0.8f;  // Base throttle magnitude
    
    int32_t currentPosition = readEncoder();
    if (reset) {
        baseEncoderPosition = currentPosition;
        movingForward = true;  // Start by moving forward
    }
    
    int32_t delta = currentPosition - baseEncoderPosition;
    
    // Switch direction based on current position
    if (movingForward && delta >= danceAmplitude) {
        movingForward = false;
    } else if (!movingForward && delta <= -danceAmplitude) {
        movingForward = true;
    }
    
    // Set throttle based on direction
    *throttle = movingForward ? throttleStrength : -throttleStrength;
    *steering = movingForward ? omega : -omega;
}

void applyMotorControl(const MotorControl* control) {
    static bool differentialInitialized = false;
    
    if (control == NULL) return;
    
    float throttle, steering;
    
    // Check if omega is non-zero (use differential drive mode)
    if (control->omega != 0.0f) {
        // Use differential to Ackermann conversion
        differentialToAckermann(control->omega, !differentialInitialized, &throttle, &steering);
        differentialInitialized = true;
    } else {
        // Use direct steering and throttle commands
        throttle = control->throttle;
        steering = control->steering;
        differentialInitialized = false;  // Reset for next differential mode entry
    }
    
    // Apply to actuators
    Turning_SetAngle(steering);
    SetEscSpeed(throttle);
}

bool isWaypointReachable(float car_x, float car_y, float car_yaw, float wp_x, float wp_y) {
    // Calculate vector from car to waypoint
    float dx = wp_x - car_x;
    float dy = wp_y - car_y;
    
    // Calculate angle to waypoint in degrees
    float angle_to_waypoint = atan2f(dy, dx) * 180.0f / M_PI;
    
    // Normalize to 0-360 range
    while (angle_to_waypoint < 0) angle_to_waypoint += 360.0f;
    while (angle_to_waypoint >= 360.0f) angle_to_waypoint -= 360.0f;
    
    // Normalize car yaw to 0-360 range
    float car_yaw_norm = car_yaw;
    while (car_yaw_norm < 0) car_yaw_norm += 360.0f;
    while (car_yaw_norm >= 360.0f) car_yaw_norm -= 360.0f;
    
    // Calculate relative angle (-180 to +180)
    float relative_angle = angle_to_waypoint - car_yaw_norm;
    while (relative_angle > 180.0f) relative_angle -= 360.0f;
    while (relative_angle < -180.0f) relative_angle += 360.0f;
    
    // Check if within ±22.5° (front cone) or ±22.5° from 180° (back cone)
    bool front_reachable = (relative_angle >= -22.5f && relative_angle <= 22.5f);
    bool back_reachable = (relative_angle >= 157.5f || relative_angle <= -157.5f);
    
    return front_reachable || back_reachable;
}

MotorControl computeWaypointControl(const PseudoGPS* gps, const Waypoint2D* waypoint) {
    // Default: stopped
    MotorControl control = {0.0f, 0.0f, 0.0f};
    
    // Control parameters
    const float Kp = 0.05f;
    const float maxSteering = 0.85f;
    const float waypointRadius = 5.0f;
    const float movingThrottle = 0.8f;
    
    // Validate inputs
    if (gps == NULL || waypoint == NULL) {
        return control;
    }
    
    // Get unwrapped heading angle
    float unwrappedHeading = unwrapAngle(gps->w);
    
    // Calculate distance and angle to target waypoint
    float dx = waypoint->x - gps->x;
    float dy = waypoint->y - gps->y;
    float distance = sqrtf(dx * dx + dy * dy);
    float targetAngle = atan2f(dy, dx) * 180.0f / M_PI;
    
    // Check if waypoint is reachable
    bool reachable = isWaypointReachable(gps->x, gps->y, gps->w, waypoint->x, waypoint->y);
    
    // Calculate angle error (difference between car's heading and target angle)
    float angleError = targetAngle - unwrappedHeading;
    
    // Normalize angle error to [-180, 180]
    while (angleError > 180.0f) angleError -= 360.0f;
    while (angleError < -180.0f) angleError += 360.0f;
    
    // If we're within the waypoint radius, stop
    if (distance <= waypointRadius) {
        return control;  // Already at {0, 0, 0}
    }
    
    if (reachable) {
        // Waypoint is reachable - navigate towards it
        
        // Determine if we need to reverse
        bool shouldReverse = fabsf(angleError) > 90.0f;
        
        // When reversing, flip the angle error by 180° to point backwards
        if (shouldReverse) {
            if (angleError > 0.0f) {
                angleError -= 180.0f;
            } else {
                angleError += 180.0f;
            }
        }
        
        // Apply proportional control and invert direction
        control.steering = -Kp * angleError;
        
        // If reversing, invert steering (car steers opposite direction in reverse)
        if (shouldReverse) {
            control.steering = -control.steering;
        }
        
        // Clamp steering
        if (control.steering > maxSteering) control.steering = maxSteering;
        if (control.steering < -maxSteering) control.steering = -maxSteering;
        
        // Set throttle
        if (shouldReverse) {
            control.throttle = -movingThrottle;  // Reverse
        } else {
            control.throttle = movingThrottle;   // Forward
        }
        
    } else {
        // Waypoint is unreachable - spin in place to align
        
        // Calculate rotation needed to point front at waypoint
        float frontAngle = targetAngle - gps->w;
        while (frontAngle > 180.0f) frontAngle -= 360.0f;
        while (frontAngle < -180.0f) frontAngle += 360.0f;
        
        // Calculate rotation needed to point back at waypoint (add 180°)
        float backAngle = frontAngle + 180.0f;
        if (backAngle > 180.0f) backAngle -= 360.0f;
        if (backAngle < -180.0f) backAngle += 360.0f;
        
        // Choose the smaller rotation (front or back alignment)
        float relativeAngle;
        if (fabsf(frontAngle) <= fabsf(backAngle)) {
            relativeAngle = frontAngle;  // Align front to waypoint
        } else {
            relativeAngle = backAngle;   // Align back to waypoint
        }
        
        // Use differential drive to spin
        // Positive relativeAngle = waypoint is CCW, use negative omega (inverted)
        // Negative relativeAngle = waypoint is CW, use positive omega (inverted)
        control.omega = (relativeAngle > 0.0f) ? -0.7f : 0.7f;
    }
    
    return control;
}

// LOOP FUNCTIONS
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

void loopSpinInPlace(void) {
    const float omega = 0.7f;  // Spinning factor
    bool firstCall = true;
    
    printf("\n\r[Spin In Place] Starting with omega=%.2f...\n\r", omega);
    
    while (1) {
        float throttle, steering;
        
        // Call differential to Ackermann converter
        differentialToAckermann(omega, firstCall, &throttle, &steering);
        firstCall = false;
        
        // Apply to actuators
        Turning_SetAngle(steering);
        SetEscSpeed(throttle);
        
        // Drain CAN messages
        drainAndUpdateCANMessages();
        
        HAL_Delay(10);
    }
}

void loopPrintAll(void *argument) {
    const uint32_t printIntervalMs = 100;
    const uint32_t heartbeatIntervalMs = 1000;
    uint32_t lastPrint = HAL_GetTick();
    uint32_t lastHeartbeat = HAL_GetTick();

    printf("\n\r[Combined Monitor] Starting...\n\r");

    while (1)
    {
        uint32_t now = HAL_GetTick();

        // Drain CAN FIFO once per loop iteration
        drainAndUpdateCANMessages();

        // Read current position
        PseudoGPS gps;
        bool hasGPS = readPseudoGPS(&gps);

        // Read motor control and waypoint commands
        MotorControl control = {0.0f, 0.0f, 0.0f};
        readMotorControl(&control);
        
        Waypoint2D waypoint;
        readWaypoint2D(&waypoint);

        // Apply control based on current mode
        if (currentMode == MODE_WAYPOINT && hasGPS) {
            // Navigate to waypoint
            control = computeWaypointControl(&gps, &waypoint);
        }
        
        // Apply motor control (handles both manual and waypoint modes)
        applyMotorControl(&control);

        // 2) Periodically print all values
        if (now - lastPrint >= printIntervalMs)
        {
            int32_t encoderCount = readEncoder();

            printf("Enc:%8ld | ", (long)encoderCount);

            if (hasGPS) {
                printf("GPS X:%5.0f Y:%5.0f W:%5.0f | ", gps.x, gps.y, gps.w);
            } else {
                printf("GPS:[No data] | ");
            }

            printf("T:%+5.2f S:%+5.2f W:%+5.2f | ", control.throttle, control.steering, control.omega);
            printf("WP X:%5.1f Y:%5.1f O:%5.1f | ", waypoint.x, waypoint.y, waypoint.omega);
            printf("Mode:%s\n\r", (currentMode == MODE_MANUAL) ? "MAN" : "WAY");

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

void navigationLoop(void) {
    const uint32_t printIntervalMs = 1000;
    uint32_t lastPrint = HAL_GetTick();
    
    // Default waypoint: center position (90, 60)
    Waypoint2D target = {90.0f, 60.0f, 0.0f};
    
    printf("\n\r[Navigation Loop] Starting with default waypoint (90, 60)...\n\r");
    
    while (1) {
        uint32_t now = HAL_GetTick();
        
        // Drain CAN FIFO once per loop iteration
        drainAndUpdateCANMessages();
        
        // Read current position
        PseudoGPS gps;
        bool hasGPS = readPseudoGPS(&gps);
        
        // Check for external waypoint commands (overrides default)
        Waypoint2D externalWaypoint;
        if (readWaypoint2D(&externalWaypoint)) {
            target = externalWaypoint;  // Override with external waypoint
        }
        
        // Compute navigation control
        MotorControl control = {0.0f, 0.0f, 0.0f};
        if (hasGPS) {
            control = computeWaypointControl(&gps, &target);
        }
        
        // Apply motor control (handles both direct and differential drive modes)
        applyMotorControl(&control);
        
        // LED: ON when using omega (unreachable), OFF otherwise
        bool spinning = (control.omega != 0.0f);
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, spinning ? GPIO_PIN_SET : GPIO_PIN_RESET);
        
        // Periodic debug print
        if (now - lastPrint >= printIntervalMs) {
            if (hasGPS) {
                float dx = target.x - gps.x;
                float dy = target.y - gps.y;
                float distance = sqrtf(dx * dx + dy * dy);
                
                printf("Pos: (%.1f, %.1f) Heading: %.1f° | Target: (%.1f, %.1f) | Dist: %.1f | Thr: %+.2f Str: %+.3f Omg: %+.2f\n\r",
                       gps.x, gps.y, gps.w, target.x, target.y, distance, control.throttle, control.steering, control.omega);
            } else {
                printf("GPS: [No data]\n\r");
            }
            lastPrint = now;
        }
        
        HAL_Delay(10);  // 100 Hz update rate
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
  SetEscSpeed(0);
  HAL_Delay(500);
//  loopPrintAllCAN();
//  loopPrintAll();
//    navigationLoop();
//  loopSpinInPlace();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(loopPrintAll, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

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

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
