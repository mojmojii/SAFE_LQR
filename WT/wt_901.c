#include "wt_901.h"
#include "usart.h"
#include <string.h>

#define WT901_FRAME_ACCELERATION     0x51U
#define WT901_FRAME_ANGULAR_VELOCITY 0x52U
#define WT901_FRAME_ANGLE            0x53U

#define WT901_ACCELERATION_SCALE (16.0f / 32768.0f)
#define WT901_GYROSCOPE_SCALE    (2000.0f / 32768.0f)
#define WT901_ANGLE_SCALE        (180.0f / 32768.0f)
#define WT901_TEMPERATURE_SCALE  (1.0f / 100.0f)

static int16_t WT901_ReadInt16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static bool WT901_IsChecksumValid(const uint8_t *frame)
{
    uint8_t checksum = 0;

    for (uint8_t index = 0; index < WT901_FRAME_LENGTH - 1U; index++)
    {
        checksum = (uint8_t)(checksum + frame[index]);
    }

    return checksum == frame[WT901_FRAME_LENGTH - 1U];
}

static void WT901_ParseFrame(WT901_Handle_t *handle, const uint8_t *frame)
{
    const int16_t value_x = WT901_ReadInt16(&frame[2]);
    const int16_t value_y = WT901_ReadInt16(&frame[4]);
    const int16_t value_z = WT901_ReadInt16(&frame[6]);
    const int16_t temperature = WT901_ReadInt16(&frame[8]);

    switch (frame[1])
    {
        case WT901_FRAME_ACCELERATION:
            handle->data.acceleration_g[0] = value_x * WT901_ACCELERATION_SCALE;
            handle->data.acceleration_g[1] = value_y * WT901_ACCELERATION_SCALE;
            handle->data.acceleration_g[2] = value_z * WT901_ACCELERATION_SCALE;
            handle->data.update_flags |= WT901_UPDATE_ACCELERATION;
            handle->data.temperature_c = temperature * WT901_TEMPERATURE_SCALE;
            break;

        case WT901_FRAME_ANGULAR_VELOCITY:
            handle->data.angular_velocity_dps[0] = value_x * WT901_GYROSCOPE_SCALE;
            handle->data.angular_velocity_dps[1] = value_y * WT901_GYROSCOPE_SCALE;
            handle->data.angular_velocity_dps[2] = value_z * WT901_GYROSCOPE_SCALE;
            handle->data.update_flags |= WT901_UPDATE_ANGULAR_VELOCITY;
            handle->data.temperature_c = temperature * WT901_TEMPERATURE_SCALE;
            break;

        case WT901_FRAME_ANGLE:
            handle->data.angle_deg[0] = value_x * WT901_ANGLE_SCALE;
            handle->data.angle_deg[1] = value_y * WT901_ANGLE_SCALE;
            handle->data.angle_deg[2] = value_z * WT901_ANGLE_SCALE;
            handle->data.update_flags |= WT901_UPDATE_ANGLE;
            break;

        default:
            return;
    }

    handle->data.valid_frame_count++;

    if (handle->callback != NULL)
    {
        handle->callback((const WT901_Data_t *)&handle->data);
    }
}

HAL_StatusTypeDef WT901_Init(WT901_Handle_t *handle, UART_HandleTypeDef *huart)
{
    if (handle == NULL || huart == NULL || huart->hdmarx == NULL)
    {
        return HAL_ERROR;
    }

    if (huart->hdmarx->Init.Mode != DMA_CIRCULAR)
    {
        return HAL_ERROR;
    }

    handle->uart = huart;
    handle->frame_index = 0;
    handle->dma_position = 0;
    handle->callback = NULL;
    memset((void *)&handle->data, 0, sizeof(handle->data));

    return WT901_StartReceive(handle);
}

HAL_StatusTypeDef WT901_StartReceive(WT901_Handle_t *handle)
{
    HAL_StatusTypeDef status;

    if (handle == NULL || handle->uart == NULL || handle->uart->hdmarx == NULL)
    {
        return HAL_ERROR;
    }

    status = HAL_UART_Receive_DMA(
        handle->uart,
        handle->dma_buffer,
        sizeof(handle->dma_buffer));

    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(handle->uart->hdmarx, DMA_IT_HT);
        __HAL_DMA_ENABLE_IT(handle->uart->hdmarx, DMA_IT_TC | DMA_IT_TE);
        __HAL_UART_CLEAR_IDLEFLAG(handle->uart);
        __HAL_UART_ENABLE_IT(handle->uart, UART_IT_IDLE);
        __HAL_UART_ENABLE_IT(handle->uart, UART_IT_ERR);
    }

    return status;
}

static void WT901_ProcessBytes(WT901_Handle_t *handle, const uint8_t *data, uint16_t length)
{
    if (handle == NULL || data == NULL)
    {
        return;
    }

    for (uint16_t index = 0; index < length; index++)
    {
        const uint8_t byte = data[index];

        if (handle->frame_index == 0U)
        {
            if (byte == WT901_FRAME_HEADER)
            {
                handle->frame[handle->frame_index++] = byte;
            }
            continue;
        }

        handle->frame[handle->frame_index++] = byte;

        if (handle->frame_index == WT901_FRAME_LENGTH)
        {
            if (WT901_IsChecksumValid(handle->frame))
            {
                WT901_ParseFrame(handle, handle->frame);
            }
            else
            {
                handle->data.checksum_error_count++;
            }

            handle->frame_index = 0;
        }
    }
}

void WT901_HandleRxEvent(WT901_Handle_t *handle, uint16_t size)
{
    if (handle == NULL)
    {
        return;
    }

    if (size > sizeof(handle->dma_buffer))
    {
        size = sizeof(handle->dma_buffer);
    }

    if (size == handle->dma_position)
    {
        return;
    }

    if (size > handle->dma_position)
    {
        WT901_ProcessBytes(
            handle,
            &handle->dma_buffer[handle->dma_position],
            size - handle->dma_position);
    }
    else
    {
        WT901_ProcessBytes(
            handle,
            &handle->dma_buffer[handle->dma_position],
            sizeof(handle->dma_buffer) - handle->dma_position);
        WT901_ProcessBytes(handle, handle->dma_buffer, size);
    }

    handle->dma_position = size;

    if (handle->dma_position == sizeof(handle->dma_buffer))
    {
        handle->dma_position = 0;
    }
}

void WT901_HandleIdleInterrupt(WT901_Handle_t *handle)
{
    uint16_t rx_count;

    if (handle == NULL || handle->uart == NULL || handle->uart->hdmarx == NULL)
    {
        return;
    }

    rx_count = sizeof(handle->dma_buffer) - __HAL_DMA_GET_COUNTER(handle->uart->hdmarx);

    WT901_HandleRxEvent(handle, rx_count);
}

void WT901_HandleError(WT901_Handle_t *handle)
{
    if (handle == NULL || handle->uart == NULL)
    {
        return;
    }

    HAL_UART_AbortReceive(handle->uart);
    handle->dma_position = 0;
    WT901_StartReceive(handle);
}

bool WT901_Read(WT901_Handle_t *handle, WT901_Data_t *data)
{
    uint32_t interrupt_state;
    bool has_update;

    if (handle == NULL || data == NULL)
    {
        return false;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();

    has_update = handle->data.update_flags != 0U;
    memcpy(data, (const void *)&handle->data, sizeof(*data));
    handle->data.update_flags = 0U;

    if (interrupt_state == 0U)
    {
        __enable_irq();
    }

    return has_update;
}

void WT901_SetCallback(WT901_Handle_t *handle, WT901_Callback_t callback)
{
    if (handle != NULL)
    {
        handle->callback = callback;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}