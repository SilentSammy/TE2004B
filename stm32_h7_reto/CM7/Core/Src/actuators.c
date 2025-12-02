/*
 * actuators.c
 *
 *  Created on: Nov 28, 2025
 *      Author: abrah
 */

#include "robot_types.h"
#include "main.h"
#include "actuators.h"

extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

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
