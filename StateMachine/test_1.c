#include "test_1.h"
#include "wt_901.h"
#include "main.h"
#include "pid.h"
#include "bsp_can.h"
#include "struct_typedef.h"
#include "cmsis_os.h"
#include "task.h"
#include "tim.h"
#include "test_2.h"
#include "oled_i2c.h"

WT901_Handle_t wt901_handle;
WT901_Handle_t wt901_handle_2;
volatile uint32_t tim2_interrupt_count = 0U;
volatile uint32_t pa0_interrupt_count = 0U;
volatile uint32_t pa1_interrupt_count = 0U;
volatile uint32_t pa2_interrupt_count = 0U;
volatile uint32_t pi5_interrupt_count = 0U;
volatile uint32_t pi6_interrupt_count = 0U;
volatile uint32_t pi7_interrupt_count = 0U;
volatile uint8_t oled_timing_enabled = 0U;
volatile uint8_t tracking_enabled = 0U;
volatile uint8_t stopping_enabled = 0U;
volatile uint32_t stop_start_ticks = 0U;
volatile uint8_t stop_delay_enabled = 0U;
volatile uint32_t stop_delay_start_ticks = 0U;
volatile uint8_t immediate_stop_enabled = 0U;
volatile uint8_t oled_stop_flash_requested = 0U;
volatile uint32_t oled_time_ticks = 0U; /* TIM2 is configured for 10 Hz (0.1 s/tick). */
volatile uint8_t tracking_run_mode = 0U; /* 1: PA3 normal, 2: PI5 timed run. */
volatile uint8_t lqr_adjustment_active = 0U;
volatile uint8_t lqr_adjustment_confirmed = 0U;
volatile int32_t lqr_adjustment_half_cm = 0;
volatile int32_t lqr_adjustment_final_quarter_cm = 0;
volatile uint32_t lqr_adjustment_confirm_until = 0U;
volatile uint8_t lqr_adjustment_flash_complete = 0U;
static volatile uint8_t lqr_adjustment_target_pending = 0U;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
        case GPIO_PIN_0:
            pa0_interrupt_count++;
            if (lqr_adjustment_active)
            {
                lqr_adjustment_active = 0U;
                lqr_adjustment_confirmed = 1U;
                lqr_adjustment_confirm_until = HAL_GetTick() + 2000U;
                lqr_adjustment_final_quarter_cm = lqr_adjustment_half_cm * 2;
                if (lqr_adjustment_final_quarter_cm > 0) lqr_adjustment_final_quarter_cm++;
                else if (lqr_adjustment_final_quarter_cm < 0) lqr_adjustment_final_quarter_cm--;
                lqr_adjustment_target_pending = 1U;
                lqr_adjustment_flash_complete = 0U;
                oled_stop_flash_requested = 1U;
            }
            else
            {
                /* Before GPIO7 adjustment mode, GPIO0 keeps its original center/stop action. */
                lqr_target_command = LQR_TARGET_CENTER;
            }
            break;
        case GPIO_PIN_1:
            pa1_interrupt_count++;
            oled_time_ticks = 0U;
            oled_timing_enabled = 1U;
            oled_stop_flash_requested = 0U;
            lqr_target_command = LQR_TARGET_PIN1_SEQUENCE;
            break;
        case GPIO_PIN_2:
            pa2_interrupt_count++;
            if (lqr_adjustment_active)
            {
                lqr_adjustment_half_cm--;
            }
            break;
        case GPIO_PIN_3:
            oled_time_ticks = 0U;
            oled_timing_enabled = 1U;
            tracking_enabled = 1U;
            stopping_enabled = 0U;
            stop_delay_enabled = 0U;
            immediate_stop_enabled = 0U;
            tracking_run_mode = 1U;
            break;
        case GPIO_PIN_5:
            pi5_interrupt_count++;
            oled_time_ticks = 0U;
            oled_timing_enabled = 1U;
            tracking_enabled = 1U;
            stopping_enabled = 0U;
            stop_delay_enabled = 0U;
            immediate_stop_enabled = 0U;
            tracking_run_mode = 2U;
            break;
        case GPIO_PIN_6:
            pi6_interrupt_count++;
            oled_time_ticks = 0U;
            oled_timing_enabled = 1U;
            tracking_enabled = 1U;
            stopping_enabled = 0U;
            stop_delay_enabled = 0U;
            immediate_stop_enabled = 0U;
            tracking_run_mode = 3U;
            break;
        case GPIO_PIN_7:
            pi7_interrupt_count++;
            if (!lqr_adjustment_active)
            {
                lqr_adjustment_active = 1U;
                lqr_adjustment_confirmed = 0U;
                lqr_adjustment_half_cm = 0;
                lqr_adjustment_final_quarter_cm = 0;
                lqr_adjustment_flash_complete = 0U;
                lqr_adjustment_target_pending = 0U;
            }
            lqr_adjustment_half_cm++;
            break;
        default:
            break;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        tim2_interrupt_count++;
        if (oled_timing_enabled)
        {
            oled_time_ticks++;
        }
    }

    if (htim->Instance == TIM7)
    {
        HAL_IncTick();
    }
}

// static void WT901_DataCallback_1(const WT901_Data_t *data)
// {
//     if (data->update_flags & WT901_UPDATE_ANGULAR_VELOCITY)
//     {
//         float gx = data->angular_velocity_dps[0];
//         float gy = data->angular_velocity_dps[1];
//         float gz = data->angular_velocity_dps[2];
//         (void)gx;
//         (void)gy;
//         (void)gz;
//     }
// }

// static void WT901_DataCallback_2(const WT901_Data_t *data)
// {
//     if (data->update_flags & WT901_UPDATE_ANGULAR_VELOCITY)
//     {
//         float gx = data->angular_velocity_dps[0];
//         float gy = data->angular_velocity_dps[1];
//         float gz = data->angular_velocity_dps[2];
//         (void)gx;
//         (void)gy;
//         (void)gz;
//     }
// }

static uint8_t WT901_ReadTrackingSensors(void)
{
    uint8_t value = 0;

    if (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_0) == GPIO_PIN_SET) { value |= (1U << 0); }
    if (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_12) == GPIO_PIN_SET) { value |= (1U << 1); }
    if (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_11) == GPIO_PIN_SET) { value |= (1U << 2); }
    if (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10) == GPIO_PIN_SET) { value |= (1U << 3); }
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == GPIO_PIN_SET) { value |= (1U << 4); }
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_SET) { value |= (1U << 5); }
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_SET) { value |= (1U << 6); }
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_SET) { value |= (1U << 7); }

    return value;
}

static fp32 Tracking_CalcError(uint8_t sensors)
{
    static const fp32 pa3_sensor_weight[8] = {
        -18.5f, -17.5f, -12.5f, -3.5f, 3.5f, 12.5f, 17.5f, 18.5f
    };
    static const fp32 pi5_pi6_sensor_weight[8] = {
        -25.0f, -18.5f, -10.0f, -3.5f, 3.5f, 10.0f, 18.5f, 25.0f
    };
    const fp32 *sensor_weight = (tracking_run_mode == 1U)
                                    ? pa3_sensor_weight
                                    : pi5_pi6_sensor_weight;
    fp32 sum = 0.0f;
    int count = 0;

    if (tracking_run_mode == 1U)
    {
        /* PA3: use only the outermost active point in groups 1-3 and 6-8. */
        int left = (sensors & (1U << 0)) ? 0 :
                   ((sensors & (1U << 1)) ? 1 :
                   ((sensors & (1U << 2)) ? 2 : -1));
        int right = (sensors & (1U << 7)) ? 7 :
                    ((sensors & (1U << 6)) ? 6 :
                    ((sensors & (1U << 5)) ? 5 : -1));

        if (left >= 0) { sum += sensor_weight[left]; count++; }
        if (sensors & (1U << 3)) { sum += sensor_weight[3]; count++; }
        if (sensors & (1U << 4)) { sum += sensor_weight[4]; count++; }
        if (right >= 0) { sum += sensor_weight[right]; count++; }
    }
    else
    {
        for (int i = 0; i < 8; i++)
        {
            if (sensors & (1U << i))
            {
                sum += sensor_weight[i];
                count++;
            }
        }
    }

    if (count == 0)
    {
        return 0.0f;
    }

    fp32 error = sum / (fp32)count;
    if (tracking_run_mode == 1U)
    {
        if (error > 17.5f) error = 17.5f;
        if (error < -17.5f) error = -17.5f;
    }
    return error;
}

static fp32 Tracking_LowPassFilter(fp32 input, fp32 prev_output)
{
    const fp32 alpha = 0.1f;
    return alpha * input + (1.0f - alpha) * prev_output;
}


const static fp32 PID[3] = {7.30f, 0.8f, 1.3f};
pid_type_def chasis_pid[4];

static int16_t Motor_OutputWithFeedforward(fp32 pid_out, fp32 feedforward)
{
    fp32 output = pid_out + feedforward;
    if (output > 12000.0f) output = 12000.0f;
    if (output < -12000.0f) output = -12000.0f;
    return (int16_t)output;
}

static void LQR_AdjustmentProcess(void)
{
    if (lqr_adjustment_target_pending && lqr_adjustment_flash_complete)
    {
        lqr_adjustable_target = (float)lqr_adjustment_final_quarter_cm * 0.0025f;
        lqr_target_command = LQR_TARGET_ADJUSTABLE;
        lqr_adjustment_target_pending = 0U;
        lqr_adjustment_flash_complete = 0U;
    }
}

void StartTask_1(void const *pvParameters) {
    uint8_t tracking;
    uint8_t dark_count;
    fp32 tracking_error;
    fp32 tracking_out;
    fp32 tracking_filtered_out = 0.0f;
    fp32 ramp_base_speed;
    fp32 feedforward;
    const fp32 tracking_kp = 1.0f;
    fp32 base_speed;
    uint8_t last_run_mode = 0U;

    // WT901_Init(&wt901_handle, &huart8);
    // WT901_SetCallback(&wt901_handle, WT901_DataCallback_1);

    // WT901_Init(&wt901_handle_2, &huart6);
    // WT901_SetCallback(&wt901_handle_2, WT901_DataCallback_2);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);
    can_filter_init();
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
    {
        Error_Handler();
    }


    for (int i = 0; i < 4; i++)
    {
        PID_init(&chasis_pid[i], PID_POSITION, PID, 12000.0f, 600.0f, 20);
    }

    while (1) {
        LQR_AdjustmentProcess();
        if (tracking_run_mode != last_run_mode)
        {
            tracking_filtered_out = 0.0f;
            last_run_mode = tracking_run_mode;
        }
        if (tracking_run_mode == 1U)
        {
            base_speed = 3600.0f; /* PA3 parameters. */
        }
        else
        {
            base_speed = (tracking_run_mode == 2U) ? 3600.0f : 5500.0f; /* PI5/PI6 parameters. */
        }
        if (!tracking_enabled) {
            if (immediate_stop_enabled)
            {
                PID_calc(&chasis_pid[0], motor_chassis[0].speed_rpm, 0.0f);
                PID_calc(&chasis_pid[1], motor_chassis[1].speed_rpm, 0.0f);
                PID_calc(&chasis_pid[2], motor_chassis[2].speed_rpm, 0.0f);
                PID_calc(&chasis_pid[3], motor_chassis[3].speed_rpm, 0.0f);
                CAN_cmd_chassis(chasis_pid[0].out, chasis_pid[1].out,
                                chasis_pid[2].out, chasis_pid[3].out);
                HAL_Delay(1);
                continue;
            }
            fp32 stop_speed = 0.0f;
            uint32_t stop_elapsed = tim2_interrupt_count - stop_start_ticks;
            if (stopping_enabled && stop_elapsed < 16U)
            {
                stop_speed = base_speed * (1.0f - (fp32)stop_elapsed / 16.0f);
                if (tracking_run_mode == 1U)
                {
                    tracking = WT901_ReadTrackingSensors();
                    if (tracking != 0xFFU)
                    {
                        tracking_error = Tracking_CalcError(tracking);
                        tracking_out = tracking_kp * tracking_error;
                        tracking_filtered_out = Tracking_LowPassFilter(tracking_out, tracking_filtered_out);
                    }
                }
                else
                {
                    tracking_filtered_out = 0.0f;
                }
            }
            else
            {
                stopping_enabled = 0U;
                tracking_filtered_out = 0.0f;
            }
            feedforward = (stop_speed > 0.0f) ? 1000.0f : 0.0f;
            /* stop_speed: decelerating forward speed; tracking_filtered_out: steering correction. */
            PID_calc(&chasis_pid[0], motor_chassis[0].speed_rpm,
                     stop_speed + tracking_filtered_out * 800.0f);          /* Motor 0: forward + steering. */
            PID_calc(&chasis_pid[1], motor_chassis[1].speed_rpm,
                     -stop_speed + tracking_filtered_out * 800.0f);         /* Motor 1: reversed mounting + steering. */
            PID_calc(&chasis_pid[2], motor_chassis[2].speed_rpm,
                     -stop_speed * 0.6f + tracking_filtered_out * 1200.0f);  /* Motor 2: 60% speed, stronger steering. */
            PID_calc(&chasis_pid[3], motor_chassis[3].speed_rpm,
                     stop_speed * 0.6f + tracking_filtered_out * 1200.0f);   /* Motor 3: 60% speed, stronger steering. */
            CAN_cmd_chassis(Motor_OutputWithFeedforward(chasis_pid[0].out, -feedforward),
                            Motor_OutputWithFeedforward(chasis_pid[1].out, feedforward),
                            Motor_OutputWithFeedforward(chasis_pid[2].out, feedforward * 0.6f),
                            Motor_OutputWithFeedforward(chasis_pid[3].out, -feedforward * 0.6f));
            HAL_Delay(1);
            continue;
        }

        tracking = WT901_ReadTrackingSensors();

        if (tracking_run_mode == 2U && oled_time_ticks >= 70U)
        {
            oled_timing_enabled = 0U;
            oled_stop_flash_requested = 1U;
            tracking_enabled = 0U;
            stopping_enabled = 1U;
            stop_start_ticks = tim2_interrupt_count;
            continue;
        }

        dark_count = 0U;
        for (uint8_t sensor = 0U; sensor < 8U; ++sensor)
        {
            /* Sensor low level represents a dark point. */
            if ((tracking & (1U << sensor)) == 0U) { dark_count++; }
        }
        uint32_t tracking_timeout_ticks = (tracking_run_mode == 1U) ? 300U : 200U;
        if ((tracking_run_mode == 1U || tracking_run_mode == 3U) &&
            (stop_delay_enabled || oled_time_ticks >= tracking_timeout_ticks || dark_count >= 4U))
        {
            if (tracking_run_mode == 3U)
            {
                oled_timing_enabled = 0U;
                oled_stop_flash_requested = 1U;
                tracking_enabled = 0U;
                immediate_stop_enabled = 1U;
                continue;
            }
            if (!stop_delay_enabled)
            {
                oled_timing_enabled = 0U;
                oled_stop_flash_requested = 1U;
                stop_delay_enabled = 1U;
                stop_delay_start_ticks = tim2_interrupt_count;
            }
            if ((tim2_interrupt_count - stop_delay_start_ticks) >= 5U)
            {
                stop_delay_enabled = 0U;
                tracking_enabled = 0U;
                stopping_enabled = 1U;
                stop_start_ticks = tim2_interrupt_count;
                HAL_Delay(1);
                continue;
            }
        }

        if (tracking != 0xFFU)
        {
            tracking_error = Tracking_CalcError(tracking);
            tracking_out = tracking_kp * tracking_error;
            tracking_filtered_out = Tracking_LowPassFilter(tracking_out, tracking_filtered_out);
        }

        ramp_base_speed = base_speed;
        if (oled_time_ticks < 32U) ramp_base_speed = base_speed * ((fp32)oled_time_ticks / 32.0f);
        feedforward = (oled_time_ticks < 32U) ? 500.0f : 0.0f;
        if (tracking_run_mode == 1U) {
            /* PA3 tracking parameters: edit only this block to tune PA3. */
            PID_calc(&chasis_pid[0], motor_chassis[0].speed_rpm, ramp_base_speed + tracking_filtered_out * 1500.0f);
            PID_calc(&chasis_pid[1], motor_chassis[1].speed_rpm, -ramp_base_speed + tracking_filtered_out * 1500.0f);
            PID_calc(&chasis_pid[2], motor_chassis[2].speed_rpm, -ramp_base_speed*0.7f  + 0.6f*tracking_filtered_out *1500.0f);
            PID_calc(&chasis_pid[3], motor_chassis[3].speed_rpm, ramp_base_speed *0.7f + 0.6f*tracking_filtered_out * 1500.0f);
            // PID_calc(&chasis_pid[3], motor_chassis[3].speed_rpm, motor_chassis[3].speed_rpm);
            // PID_calc(&chasis_pid[1], motor_chassis[1].speed_rpm, motor_chassis[1].speed_rpm);


        }
        else
        {
            /* PI5 and PI6 share these tracking parameters. */
            PID_calc(&chasis_pid[0], motor_chassis[0].speed_rpm, ramp_base_speed + tracking_filtered_out * 800.0f);
            PID_calc(&chasis_pid[1], motor_chassis[1].speed_rpm, -ramp_base_speed + tracking_filtered_out * 800.0f);
            PID_calc(&chasis_pid[2], motor_chassis[2].speed_rpm, -ramp_base_speed * 0.6f + tracking_filtered_out * 1200.0f);
            PID_calc(&chasis_pid[3], motor_chassis[3].speed_rpm, ramp_base_speed * 0.6f + tracking_filtered_out * 1200.0f);
        }

        CAN_cmd_chassis(Motor_OutputWithFeedforward(chasis_pid[0].out, feedforward),
                        Motor_OutputWithFeedforward(chasis_pid[1].out, -feedforward),
                        Motor_OutputWithFeedforward(chasis_pid[2].out, -feedforward * 0.6f),
                        Motor_OutputWithFeedforward(chasis_pid[3].out, feedforward * 0.6f));
        HAL_Delay(1);
    }
}

// Created by laiwt on 2026/7/30.
//
