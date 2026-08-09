#ifndef WT_901_H
#define WT_901_H

#include <stdbool.h>
#include <stdint.h>

#include "usart.h"

#define WT901_FRAME_HEADER       0x55U
#define WT901_FRAME_LENGTH       11U
#define WT901_DMA_BUFFER_LENGTH  64U

#define WT901_UPDATE_ACCELERATION     (1U << 0)
#define WT901_UPDATE_ANGULAR_VELOCITY (1U << 1)
#define WT901_UPDATE_ANGLE            (1U << 2)

typedef struct
{
    float acceleration_g[3];
    float angular_velocity_dps[3];
    float angle_deg[3];
    float temperature_c;
    uint32_t valid_frame_count;
    uint32_t checksum_error_count;
    uint8_t update_flags;
} WT901_Data_t;

typedef void (*WT901_Callback_t)(const WT901_Data_t *data);

typedef struct
{
    UART_HandleTypeDef *uart;
    uint8_t dma_buffer[WT901_DMA_BUFFER_LENGTH];
    uint8_t frame[WT901_FRAME_LENGTH];
    uint8_t frame_index;
    uint16_t dma_position;
    volatile WT901_Data_t data;
    WT901_Callback_t callback;
} WT901_Handle_t;

HAL_StatusTypeDef WT901_Init(WT901_Handle_t *handle, UART_HandleTypeDef *huart);
HAL_StatusTypeDef WT901_StartReceive(WT901_Handle_t *handle);
void WT901_HandleRxEvent(WT901_Handle_t *handle, uint16_t size);
void WT901_HandleIdleInterrupt(WT901_Handle_t *handle);
void WT901_HandleError(WT901_Handle_t *handle);
bool WT901_Read(WT901_Handle_t *handle, WT901_Data_t *data);
void WT901_SetCallback(WT901_Handle_t *handle, WT901_Callback_t callback);

#endif