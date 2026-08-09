//
// Created by laiwt on 2026/7/30.
//

#ifndef TI_EVO_TEST_1_H
#define TI_EVO_TEST_1_H
#include "main.h"
extern volatile uint32_t tim2_interrupt_count;
extern volatile uint32_t pa0_interrupt_count;
extern volatile uint32_t pa1_interrupt_count;
extern volatile uint32_t pa2_interrupt_count;
extern volatile uint32_t pi5_interrupt_count;
extern volatile uint32_t pi6_interrupt_count;
extern volatile uint32_t pi7_interrupt_count;
extern volatile uint8_t oled_timing_enabled;
extern volatile uint8_t tracking_enabled;
extern volatile uint8_t stopping_enabled;
extern volatile uint32_t stop_start_ticks;
extern volatile uint8_t stop_delay_enabled;
extern volatile uint32_t stop_delay_start_ticks;
extern volatile uint8_t immediate_stop_enabled;
extern volatile uint8_t oled_stop_flash_requested;
extern volatile uint32_t oled_time_ticks;
extern volatile uint8_t tracking_run_mode;
extern volatile uint8_t lqr_adjustment_active;
extern volatile uint8_t lqr_adjustment_confirmed;
extern volatile int32_t lqr_adjustment_half_cm;
extern volatile int32_t lqr_adjustment_final_quarter_cm;
extern volatile uint32_t lqr_adjustment_confirm_until;
extern volatile uint8_t lqr_adjustment_flash_complete;
void StartTask_1(void const *pvParameters);




#endif //TI_EVO_TEST_1_H
