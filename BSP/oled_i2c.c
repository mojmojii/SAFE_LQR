#include "oled_i2c.h"
#include "i2c.h"
#include <string.h>

static uint8_t oled_buffer[OLED_WIDTH * OLED_HEIGHT / 8U];
static uint8_t oled_page;
static uint8_t oled_column;

static HAL_StatusTypeDef oled_cmd(uint8_t cmd)
{
    uint8_t tx[2] = { 0x00U, cmd };
    return HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDR, tx, 2U, 100U);
}

static const uint8_t *glyph(char c)
{
    static uint8_t g[5];
    memset(g, 0, sizeof(g));
    switch (c) {
    case '0': g[0]=0x3e;g[1]=0x51;g[2]=0x49;g[3]=0x45;g[4]=0x3e; break;
    case '1': g[0]=0;g[1]=0x42;g[2]=0x7f;g[3]=0x40; break;
    case '2': g[0]=0x62;g[1]=0x51;g[2]=0x49;g[3]=0x49;g[4]=0x46; break;
    case '3': g[0]=0x22;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x36; break;
    case '4': g[0]=0x18;g[1]=0x14;g[2]=0x12;g[3]=0x7f;g[4]=0x10; break;
    case '5': g[0]=0x2f;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x31; break;
    case '6': g[0]=0x3e;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x32; break;
    case '7': g[0]=1;g[1]=0x71;g[2]=9;g[3]=5;g[4]=3; break;
    case '8': g[0]=0x36;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x36; break;
    case '9': g[0]=0x26;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x3e; break;
    case 'T': g[0]=1;g[1]=1;g[2]=0x7f;g[3]=1;g[4]=1; break;
    case 'A': g[0]=0x7e;g[1]=9;g[2]=9;g[3]=9;g[4]=0x7e; break;
    case 'S': g[0]=0x46;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x31; break;
    case 'K': g[0]=0x7f;g[1]=8;g[2]=0x14;g[3]=0x22;g[4]=0x41; break;
    case 'E': g[0]=0x7f;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x41; break;
    case 'I': g[0]=0x41;g[1]=0x41;g[2]=0x7f;g[3]=0x41;g[4]=0x41; break;
    case 'M': g[0]=0x7f;g[1]=2;g[2]=4;g[3]=2;g[4]=0x7f; break;
    case 'R': g[0]=0x7f;g[1]=9;g[2]=0x19;g[3]=0x29;g[4]=0x46; break;
    case ':': g[1]=0x36;g[3]=0x36; break;
    case '.': g[2]=0x60; break;
    case '+': g[0]=0x08;g[1]=0x08;g[2]=0x3e;g[3]=0x08;g[4]=0x08; break;
    case '-': g[0]=0x08;g[1]=0x08;g[2]=0x08;g[3]=0x08;g[4]=0x08; break;
    case ' ': break;
    default: g[0]=g[1]=g[2]=g[3]=g[4]=0x08; break;
    }
    return g;
}

HAL_StatusTypeDef OLED_Init(void)
{
    HAL_Delay(20);
    static const uint8_t init[] = {0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,0x8D,0x14,0x20,0x00,0xA1,0xC8,0xDA,0x12,0x81,0x7F,0xD9,0xF1,0xDB,0x40,0xA4,0xA6,0xAF};
    for (uint32_t i=0; i<sizeof(init); ++i) if (oled_cmd(init[i]) != HAL_OK) return HAL_ERROR;
    return OLED_Clear();
}

HAL_StatusTypeDef OLED_Clear(void) { memset(oled_buffer, 0, sizeof(oled_buffer)); return OLED_Update(); }

HAL_StatusTypeDef OLED_Update(void)
{
    uint8_t tx[OLED_WIDTH + 1U];
    for (uint8_t page=0; page<8U; ++page) {
        if (oled_cmd((uint8_t)(0xB0U + page)) != HAL_OK || oled_cmd(0x00U) != HAL_OK || oled_cmd(0x10U) != HAL_OK) return HAL_ERROR;
        tx[0]=0x40U; memcpy(&tx[1], &oled_buffer[page*OLED_WIDTH], OLED_WIDTH);
        if (HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDR, tx, sizeof(tx), 100U) != HAL_OK) return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef OLED_SetInverted(uint8_t inverted)
{
    return oled_cmd(inverted ? 0xA7U : 0xA6U);
}

void OLED_SetCursor(uint8_t page, uint8_t column) { if(page<8U && column<OLED_WIDTH) { oled_page=page; oled_column=column; } }
void OLED_ShowChar(char c) { const uint8_t *p=glyph(c); if(oled_column>122U){oled_column=0;oled_page=(uint8_t)((oled_page+1U)%8U);} for(uint8_t i=0;i<5U;i++) oled_buffer[oled_page*OLED_WIDTH+oled_column+i]=p[i]; oled_buffer[oled_page*OLED_WIDTH+oled_column+5U]=0; oled_column=(uint8_t)(oled_column+6U); }
void OLED_ShowString(const char *s) { while(*s) OLED_ShowChar(*s++); }
void OLED_ShowUInt(uint32_t value) { char b[11]; uint8_t i=0; if(value==0) { OLED_ShowChar('0'); return; } while(value && i<10U){b[i++]=(char)('0'+value%10U);value/=10U;} while(i) OLED_ShowChar(b[--i]); }

void OLED_ShowCharLarge(char c)
{
    const uint8_t *p = glyph(c);
    if (oled_page > 6U) oled_page = 6U;
    if (oled_column > 116U) { oled_column = 0U; oled_page = (uint8_t)((oled_page + 2U) % 8U); }
    for (uint8_t x = 0; x < 6U; ++x) {
        uint8_t source = (x < 5U) ? p[x] : 0U;
        uint16_t scaled = 0U;
        for (uint8_t y = 0; y < 8U; ++y) {
            if (source & (1U << y)) scaled |= (uint16_t)(3U << (y * 2U));
        }
        oled_buffer[oled_page * OLED_WIDTH + oled_column + x * 2U] = (uint8_t)scaled;
        oled_buffer[oled_page * OLED_WIDTH + oled_column + x * 2U + 1U] = (uint8_t)scaled;
        oled_buffer[(oled_page + 1U) * OLED_WIDTH + oled_column + x * 2U] = (uint8_t)(scaled >> 8U);
        oled_buffer[(oled_page + 1U) * OLED_WIDTH + oled_column + x * 2U + 1U] = (uint8_t)(scaled >> 8U);
    }
    oled_column = (uint8_t)(oled_column + 12U);
}

void OLED_ShowUIntLarge(uint32_t value)
{
    char b[11]; uint8_t i = 0U;
    if (value == 0U) { OLED_ShowCharLarge('0'); return; }
    while (value && i < 10U) { b[i++] = (char)('0' + value % 10U); value /= 10U; }
    while (i) OLED_ShowCharLarge(b[--i]);
}

/* 16x16 cat face with tongue, placed by the caller at the lower-right corner. */
void OLED_DrawCat(void)
{
    static const uint8_t top[16] = {
        0x00,0x60,0xF0,0xF8,0xFC,0xFE,0xFE,0xFF,
        0xFF,0xFE,0xFE,0xFC,0xF8,0xF0,0x60,0x00
    };
    static const uint8_t bottom[16] = {
        0x00,0x03,0x07,0x0F,0x1F,0x3F,0x3F,0x7F,
        0x7F,0x3F,0x3F,0x1F,0x0F,0x07,0x03,0x00
    };
    const uint8_t x = 112U;
    for (uint8_t i = 0; i < 16U; ++i) {
        oled_buffer[6U * OLED_WIDTH + x + i] = top[i];
        oled_buffer[7U * OLED_WIDTH + x + i] = bottom[i];
    }
    /* eyes, nose, and the protruding tongue */
    oled_buffer[6U * OLED_WIDTH + x + 4U] = 0xF0U;
    oled_buffer[6U * OLED_WIDTH + x + 11U] = 0xF0U;
    oled_buffer[7U * OLED_WIDTH + x + 7U] = 0x3FU;
    oled_buffer[7U * OLED_WIDTH + x + 8U] = 0x3FU;
}
