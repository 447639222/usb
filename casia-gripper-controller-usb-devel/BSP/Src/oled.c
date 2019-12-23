/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       oled.c
 * @brief      this file contains sd card basic operating function
 * @note
 * @Version    V1.0.0
 * @Date       Jan-28-2018
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#include "oled.h"
#include "oledfont.h"
#include "math.h"
#include <stdio.h>
#include <stdarg.h>

/**
 * OLED flash Addr:
 * [0]0 1 2 3 ... 127
 * [1]0 1 2 3 ... 127
 * [2]0 1 2 3 ... 127
 * [3]0 1 2 3 ... 127
 * [4]0 1 2 3 ... 127
 * [5]0 1 2 3 ... 127
 * [6]0 1 2 3 ... 127
 * [7]0 1 2 3 ... 127
 **/

static uint8_t OLED_GRAM[132][8];

/**
 * @brief   write data/command to OLED
 * @param   dat: the data ready to write
 * @param   cmd: 0x00,command 0x01,data
 * @retval
 */
void oled_write_byte(uint8_t dat, uint8_t cmd)
{
    if (cmd != 0)
        OLED_CMD_Set();
    else
        OLED_CMD_Clr();

    HAL_SPI_Transmit(&hspi1, &dat, 1, 10);
}

/**
 * @brief   set OLED cursor position
 * @param   x: the X-axis of cursor
 * @param   y: the Y-axis of cursor
 * @retval
 */
static void oled_set_pos(uint8_t x, uint8_t y)
{
    x += 2;
    oled_write_byte((0xb0 + y), OLED_CMD);              //set page address y
    oled_write_byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD); //set column high address
    oled_write_byte((x & 0xf0), OLED_CMD);              //set column low address
}

/**
 * @brief   turn on OLED display
 * @param   None
 * @param   None
 * @retval
 */
void oled_display_on(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x14, OLED_CMD);
    oled_write_byte(0xaf, OLED_CMD);
}

/**
 * @brief   turn off OLED display
 * @param   None
 * @param   None
 * @retval
 */
void oled_display_off(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x10, OLED_CMD);
    oled_write_byte(0xae, OLED_CMD);
}

/**
 * @brief   refresh the RAM of OLED
 * @param   None
 * @param   None
 * @retval
 */
void oled_refresh_gram(void)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        oled_set_pos(0, i);
        for (n = 0; n < 132; n++)
        {
            oled_write_byte(OLED_GRAM[n][i], OLED_DATA);
        }
    }
}

/**
 * @brief   clear the screen
 * @param   None
 * @param   None
 * @retval
 */
void oled_clear(Pen_Typedef pen)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        for (n = 0; n < 132; n++)
        {
            if (pen == Pen_Write)
                OLED_GRAM[n][i] = 0xff;
            else if (pen == Pen_Clear)
                OLED_GRAM[n][i] = 0x00;
            else
                OLED_GRAM[n][i] = 0xff - OLED_GRAM[n][i];
        }
    }
}

/**
 * @brief   draw a point at (x, y)
 * @param   x: the X-axis of cursor
 * @param   y: the Y-axis of cursor
 * @param   pen: Pen_Clear, Pen_Write, Pen_Inversion @Pen_Typedef
 * @retval  None
 */
void oled_drawpoint(int8_t x, int8_t y, Pen_Typedef pen)
{
    uint8_t page = 0, row = 0;

    /* check the corrdinate */
    if ((x < 0) || (x > (X_WIDTH - 1)) || (y < 0) || (y > (Y_WIDTH - 1)))
        return;

    page = y / 8;
    row = y % 8;

    if (pen == Pen_Write)
        OLED_GRAM[x][page] |= 1 << row;
    else if (pen == Pen_Inversion)
        OLED_GRAM[x][page] ^= 1 << row;
    else
        OLED_GRAM[x][page] &= ~(1 << row);
}

/**
 * @brief   draw a line from (x1, y1) to (x2, y2)
 * @param   x1, y1: the start point of line
 * @param   x2, y2: the end of line
 * @param   pen: Pen_Clear, Pen_Write, Pen_Inversion @Pen_Typedef
 * @retval  None
 */
void oled_drawline(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2,
        Pen_Typedef pen)
{
    uint8_t col = 0, row = 0;
    uint8_t x_st = 0, x_ed = 0, y_st = 0, y_ed = 0;
    float k = 0.0f, b = 0.0f;

    if (y1 == y2)
    {
        (x1 <= x2) ? (x_st = x1) : (x_st = x2);
        (x1 <= x2) ? (x_ed = x2) : (x_ed = x1);

        for (col = x_st; col <= x_ed; col++)
        {
            oled_drawpoint(col, y1, pen);
        }
    }
    else if (x1 == x2)
    {
        (y1 <= y2) ? (y_st = y1) : (y_st = y2);
        (y1 <= y2) ? (y_ed = y2) : (y_ed = y1);

        for (row = y_st; row <= y_ed; row++)
        {
            oled_drawpoint(x1, row, pen);
        }
    }
    else
    {
        k = ((float) (y2 - y1)) / (x2 - x1);
        b = (float) y1 - k * x1;

        (x1 <= x2) ? (x_st = x1) : (x_st = x2);
        (x1 <= x2) ? (x_ed = x2) : (x_ed = x2);

        for (col = x_st; col <= x_ed; col++)
        {
            oled_drawpoint(col, (uint8_t) (col * k + b), pen);
        }
    }
}

//To add: rectangle, fillrectangle, circle, fillcircle,

/**
 * @brief   show a character
 * @param   row: row of character
 * @param   col: column of character
 * @param   chr: the character ready to show
 * @retval  None
 */
void oled_showchar(uint8_t row, uint8_t col, uint8_t chr)
{
    uint8_t x = col * 6;
    uint8_t y = row * 12;
    uint8_t temp, t, t1;
    uint8_t y0 = y;
    chr = chr - ' ';

    for (t = 0; t < 12; t++)
    {
        temp = asc2_1206[chr][t];

        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp & 0x80)
                oled_drawpoint(x, y, Pen_Write);
            else
                oled_drawpoint(x, y, Pen_Clear);

            temp <<= 1;
            y++;
            if ((y - y0) == 12)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

//m^n
static uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;

    while (n--)
        result *= m;

    return result;
}

/**
 * @brief   show a number
 * @param   row: row of number
 * @param   col: column of number
 * @param   num: the number ready to show
 * @param   mode: 0x01, fill number with '0'; 0x00, fill number with spaces
 * @param   len: the length of the number
 * @retval  None
 */
void oled_shownum(uint8_t row, uint8_t col, uint32_t num, uint8_t mode,
        uint8_t len)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;

        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                if (mode == 0)
                    oled_showchar(row, col + t, ' ');
                else
                    oled_showchar(row, col + t, '0');
                continue;
            }
            else
                enshow = 1;
        }

        oled_showchar(row, col + t, temp + '0');
    }
}

/**
 * @brief   show a character string
 * @param   row: row of character string begin
 * @param   col: column of character string begin
 * @param   chr: the pointer to character string
 * @retval  None
 */
void oled_showstring(uint8_t row, uint8_t col, uint8_t *chr)
{
    uint8_t n = 0;

    while (chr[n] != '\0')
    {
        oled_showchar(row, col, chr[n]);
        col++;

        if (col > 20)
        {
            col = 0;
            row += 1;
        }
        n++;
    }
}

void oled_showstring1(uint8_t row, uint8_t col, char *chr)
{
    uint8_t n = 0;

    while (chr[n] != '\0')
    {
        oled_showchar(row, col, chr[n]);
        col++;

        if (col > 20)
        {
            col = 0;
            row += 1;
        }
        n++;
    }
}

// Not working
void oled_sprintf(uint8_t row, uint8_t col, const char *fmt, ...)
{
    char message[50] = "\0";

    va_list argp;
    va_start(argp, fmt);

    sprintf(message, fmt, argp);

    va_end(argp);

    oled_showstring1(row, col, message);
}



/**
 * @brief   formatted output in oled 132*64
 * @param   row: row of character string begin, 0 <= row <= 4;
 * @param   col: column of character string begin, 0 <= col <= 20;
 * @param   *fmt: the pointer to format character string
 * @retval  None
 * @note    if the character length is more than one row at a time, the extra characters will be truncated
 */
void oled_printf(uint8_t row, uint8_t col, const char *fmt, ...)
{
    uint8_t LCD_BUF[132] =
    { 0 };
    uint8_t remain_size = 0;
    va_list ap;

    if ((row > 4) || (col > 20) || (col < 1))
        return;

    va_start(ap, fmt);

    vsprintf((char*) LCD_BUF, fmt, ap);

    va_end(ap);

    remain_size = 21 - col;

    LCD_BUF[remain_size] = '\0';

    oled_showstring(row, col, LCD_BUF);
}

void oled_LOGO(void)
{
    oled_clear(Pen_Clear);
    uint8_t temp_char = 0;
    uint8_t x = 0, y = 0;
    uint8_t i = 0;
    for (; y < 64; y += 8)
    {
        for (x = 0; x < 128; x++)
        {
            temp_char = LOGO_BMP[x][y / 8];
            for (i = 0; i < 8; i++)
            {
                if (temp_char & 0x80)
                    oled_drawpoint(x, y + i, Pen_Write);
                else
                    oled_drawpoint(x, y + i, Pen_Clear);
                temp_char <<= 1;
            }
        }
    }
    oled_refresh_gram();
}

// Send single byte command to display
// input:
//   cmd - display command
static void SH1106_cmd(uint8_t cmd)
{
    // Deassert DC pin -> command transmit
    OLED_CMD_Clr();

    // Send command to display
    uint8_t command = cmd;
    HAL_SPI_Transmit(&hspi1, &command, 1, 20);
}

// Send double byte command to display
// input:
//   cmd1 - first byte of double-byte command
//   cmd2 - second byte of double-byte command
static void SH1106_cmd_double(uint8_t cmd1, uint8_t cmd2)
{
    // Deassert DC pin -> command transmit
    OLED_CMD_Clr();

    // Send double byte command to display
    uint8_t command[2];
    command[0] = cmd1;
    command[1] = cmd2;
    HAL_SPI_Transmit(&hspi1, command, 2, 20);
}

/**
 * @brief   initialize the oled module
 * @param   None
 * @retval  None
 */
void oled_init(void)
{
    // Hardware display reset
    OLED_RST_Clr();
    HAL_Delay(500);
    OLED_RST_Set();

    // Initial display configuration

    //turn off oled panel
    SH1106_cmd(SH1106_CMD_DISP_OFF);

    //set low & high column address
    SH1106_cmd(SH1106_CMD_COL_LOW);
    SH1106_cmd(SH1106_CMD_COL_HIGH);

    // Set multiplex ratio (visible lines)
    SH1106_cmd_double(SH1106_CMD_SETMUX, 0x3F); // 64MUX

    // Set display offset (offset of first line from the top of display)
    SH1106_cmd_double(SH1106_CMD_SETOFFS, 0x00); // Offset: 0

    // Set display start line (first line displayed)
    SH1106_cmd(SH1106_CMD_STARTLINE | 0x00); // Start line: 0

    // Set segment re-map (X coordinate)
    SH1106_cmd(SH1106_CMD_SEG_INV);

    // Set COM output scan direction (Y coordinate)
    SH1106_cmd(SH1106_CMD_COM_INV);

    // Set COM pins hardware configuration
    // bit[4]: reset - sequential COM pin configuration
    //         set   - alternative COM pin configuration (reset value)
    // bit[5]: reset - disable COM left/right remap (reset value)
    //         set   - enable COM left/right remap
    SH1106_cmd_double(SH1106_CMD_COM_HW, 0x12);

    //pre-charge: 15 clocks, discharge: 1 clock
    uint8_t dis_charge = 0x01;
    uint8_t pre_charge = 0x0f;
    SH1106_cmd_double(SH1106_CMD_CHARGE, dis_charge | (pre_charge << 4));

    // Set contrast control
    SH1106_cmd_double(SH1106_CMD_CONTRAST, 0xF0); // Contrast: 00H-FFH (256 levels)

    SH1106_cmd(0x30);

    // Disable entire display ON
    SH1106_cmd(SH1106_CMD_EDOFF); // Display follows RAM content

    // Disable display inversion
    SH1106_cmd(SH1106_CMD_INV_OFF); // Normal display mode

    // Set clock divide ratio and oscillator frequency
    // bits[3:0] defines the divide ratio of the display clocks (bits[3:0] + 1)
    // bits[7:4] set the oscillator frequency (Fosc), frequency increases with the value of these bits
    // 0xF0 value gives maximum frequency (maximum Fosc without divider)
    // 0x0F value gives minimum frequency (minimum Fosc divided by 16)
    // The higher display frequency decreases image flickering but increases current consumption and vice versa
    SH1106_cmd_double(SH1106_CMD_CLOCKDIV, 0x80);

    // Display ON
    SH1106_cmd(SH1106_CMD_DISP_ON); // Display enabled



//    oled_write_byte(0xdb, OLED_CMD);    //set vcomh
//    oled_write_byte(0x40, OLED_CMD);    //set vcom deselect level
//    oled_write_byte(0x20, OLED_CMD);    //set page addressing mode
//    oled_write_byte(0x02, OLED_CMD);    //
//    oled_write_byte(0x8d, OLED_CMD);    //set charge pump enable/disable
//    oled_write_byte(0x14, OLED_CMD);    //charge pump disable
//    oled_write_byte(0xa4, OLED_CMD);    //disable entire dispaly on
//    oled_write_byte(0xa6, OLED_CMD);    //disable inverse display on

    oled_clear(Pen_Clear);
    oled_set_pos(0, 0);

}

