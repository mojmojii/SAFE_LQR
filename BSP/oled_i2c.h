#ifndef OLED_I2C_H
#define OLED_I2C_H

#include "main.h"

#define OLED_I2C_ADDR       (0x3CU << 1) /* SSD1306 7-bit address 0x3C */
#define OLED_WIDTH          128U
#define OLED_HEIGHT         64U

HAL_StatusTypeDef OLED_Init(void);
HAL_StatusTypeDef OLED_Clear(void);
HAL_StatusTypeDef OLED_Update(void);
HAL_StatusTypeDef OLED_SetInverted(uint8_t inverted);
void OLED_SetCursor(uint8_t page, uint8_t column);
void OLED_ShowChar(char c);
void OLED_ShowString(const char *s);
void OLED_ShowUInt(uint32_t value);
void OLED_ShowCharLarge(char c);
void OLED_ShowUIntLarge(uint32_t value);
void OLED_DrawCat(void);

#endif
