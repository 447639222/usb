/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       oled.h
 * @brief      this file contains sd card basic operating function
 * @note
 * @Version    V1.0.0
 * @Date       Jan-28-2018
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __OLED__H
#define __OLED__H

#include "stm32f4xx.h"
#include "spi.h"
#include <stdint.h>

#define Max_Column      128
#define Max_Row         64

#define X_WIDTH         128
#define Y_WIDTH         64

#define OLED_CMD        0x00
#define OLED_DATA       0x01

#define CHAR_SIZE_WIDTH     6
#define VHAR_SIZE_HIGHT     12

#define OLED_CMD_Set()      HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET)
#define OLED_CMD_Clr()      HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET)

#define OLED_RST_Set()      HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET)
#define OLED_RST_Clr()      HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET)

// SH1106 command definitions
#define SH1106_CMD_SETMUX    (uint8_t)0xA8 // Set multiplex ratio (N, number of lines active on display)
#define SH1106_CMD_SETOFFS   (uint8_t)0xD3 // Set display offset
#define SH1106_CMD_STARTLINE (uint8_t)0x40 // Set display start line
#define SH1106_CMD_SEG_NORM  (uint8_t)0xA0 // Column 0 is mapped to SEG0 (X coordinate normal)
#define SH1106_CMD_SEG_INV   (uint8_t)0xA1 // Column 127 is mapped to SEG0 (X coordinate inverted)
#define SH1106_CMD_COM_NORM  (uint8_t)0xC0 // Scan from COM0 to COM[N-1] (N - mux ratio, Y coordinate normal)
#define SH1106_CMD_COM_INV   (uint8_t)0xC8 // Scan from COM[N-1] to COM0 (N - mux ratio, Y coordinate inverted)
#define SH1106_CMD_COM_HW    (uint8_t)0xDA // Set COM pins hardware configuration
#define SH1106_CMD_CONTRAST  (uint8_t)0x81 // Contrast control
#define SH1106_CMD_EDON      (uint8_t)0xA5 // Entire display ON enabled (all pixels on, RAM content ignored)
#define SH1106_CMD_EDOFF     (uint8_t)0xA4 // Entire display ON disabled (output follows RAM content)
#define SH1106_CMD_INV_OFF   (uint8_t)0xA6 // Entire display inversion OFF (normal display)
#define SH1106_CMD_INV_ON    (uint8_t)0xA7 // Entire display inversion ON (all pixels inverted)
#define SH1106_CMD_CLOCKDIV  (uint8_t)0xD5 // Set display clock divide ratio/oscillator frequency
#define SH1106_CMD_DISP_ON   (uint8_t)0xAF // Display ON
#define SH1106_CMD_DISP_OFF  (uint8_t)0xAE // Display OFF (sleep mode)

#define SH1106_CMD_COL_LOW   (uint8_t)0x02 // Set Lower Column Address
#define SH1106_CMD_COL_HIGH  (uint8_t)0x10 // Set Higher Column Address
#define SH1106_CMD_PAGE_ADDR (uint8_t)0xB0 // Set Page Address

#define SH1106_CMD_CHARGE    (uint8_t)0xD9 //  Dis-charge / Pre-charge Period
#define SH1106_CMD_SCRL_HR   (uint8_t)0x26 // Setup continuous horizontal scroll right
#define SH1106_CMD_SCRL_HL   (uint8_t)0x27 // Setup continuous horizontal scroll left
#define SH1106_CMD_SCRL_VHR  (uint8_t)0x29 // Setup continuous vertical and horizontal scroll right
#define SH1106_CMD_SCRL_VHL  (uint8_t)0x2A // Setup continuous vertical and horizontal scroll left
#define SH1106_CMD_SCRL_STOP (uint8_t)0x2E // Deactivate scroll
#define SH1106_CMD_SCRL_ACT  (uint8_t)0x2F // Activate scroll


typedef enum
{
    Pen_Clear = 0x00,
    Pen_Write = 0x01,
    Pen_Inversion = 0x02,
}Pen_Typedef;

/* function define */
void oled_init(void);
void oled_write_byte(uint8_t dat, uint8_t cmd);
void oled_display_on(void);
void oled_display_off(void);
void oled_refresh_gram(void);
void oled_clear(Pen_Typedef pen);
void oled_drawpoint(int8_t x, int8_t y, Pen_Typedef pen);
void oled_drawline(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, Pen_Typedef pen);
void oled_showchar(uint8_t row, uint8_t col, uint8_t chr);
void oled_shownum(uint8_t row, uint8_t col, uint32_t num, uint8_t mode, uint8_t len);
void oled_showstring(uint8_t row, uint8_t col, uint8_t *chr);
void oled_showstring1(uint8_t row, uint8_t col, char *chr);
void oled_printf(uint8_t row, uint8_t col, const char *fmt,...);
void oled_sprintf(uint8_t row, uint8_t col, const char *fmt, ...);
void oled_LOGO(void);

#endif

