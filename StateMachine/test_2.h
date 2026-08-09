//
// Created by laiwt on 2026/7/31.
//

#ifndef TI_EVO_TEST_2_H
#define TI_EVO_TEST_2_H

#include "main.h"

typedef enum
{
    LQR_TARGET_NONE = 0,
    LQR_TARGET_CENTER,
    LQR_TARGET_POSITIVE,
    LQR_TARGET_NEGATIVE,
    LQR_TARGET_PIN1_SEQUENCE,
    LQR_TARGET_ADJUSTABLE
} LQR_TargetCommand_t;

extern volatile LQR_TargetCommand_t lqr_target_command;
extern volatile float lqr_adjustable_target;

void ball_cascade_pid_init(void);
void ball_cascade_pid_reset(void);
float ball_cascade_pid_calc(float position, float motor_angle, float target_position);

void StartTask_2(void const *pvParameters);

#endif //TI_EVO_TEST_2_H
