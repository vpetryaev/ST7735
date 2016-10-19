/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@arduino.cc>
 * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
 * Copyright (c) 2014 by Matthijs Kooijman <matthijs@stdin.nl> (SPISettings AVR)
 * Copyright (c) 2014 by Andrew J. Kroll <xxxajk@gmail.com> (atomicity fixes)
 * Copyrighr (c) 2016 by Vadim Petryaev <vpetrjaev@yandex.ru> (rewriten for fast bidirectional SPI for ST7735)
 * SPI for ST4435 library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

/*-----------------10.09.2016 18:20-----------------
 * Restrictions:
 * - using only hardware SPI with polling
 * - ST7735 SPI using the same wire for sending and receiving data,
 *   so implemented support only for simplex communocation
 * - ST7735 use only MSBFIRST and SPI_MODE0
 * - D/C line always handling - mandatory for ST7735
 * --------------------------------------------------*/

#ifndef _ST7735_SPI_H_INCLUDED
#define _ST7735_SPI_H_INCLUDED

#include <stdint.h>
#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ST7735_RESET_PIN
#define ST7735_RESET_PIN        255 // Disable RESX control by library
#endif

#ifndef ST7735_BACKLIGHT_PIN
#define ST7735_BACKLIGHT_PIN    255 // Disable BL control by library
#endif

// Use 9 (PORTB1) pin as defaut.
#ifndef ST7735_CHIPSELECT_PIN
#define ST7735_CHIPSELECT_PIN   9
#endif

// Use 10 (SS, PORTB2) pin as defaut.
#ifndef ST7735_SPI_DC_PIN_NO
#define ST7735_SPI_DC_PIN_NO PIN_SPI_SS
#endif

// ST7735_SPI_USART_SEQ_NO            Supported only: 0..2
#ifndef ST7735_SPI_USART_SEQ_NO
#define ST7735_SPI_USART_SEQ_NO       0
#endif

#ifndef ST7735_SCREEN_WIDTH
#define ST7735_SCREEN_WIDTH           128
#endif
#ifndef ST7735_SCREEN_HEIGHT
#define ST7735_SCREEN_HEIGHT          160
#endif


#if PIN_SPI_SS==10 // Arduino UNO
#ifndef PIN_MSPIM_XCK
#define PIN_MSPIM_XCK 4
#endif
#define PIN_TO_PORTn_OFFSET(p) ((p<8)?_SFR_IO_ADDR(PORTD):(p<NUM_DIGITAL_PINS&&p>13)?_SFR_IO_ADDR(PORTC):(p>=NUM_DIGITAL_PINS)?65536:_SFR_IO_ADDR(PORTB))
#define PIN_TO_PINn_BIT(p) ((p<8)?p:(p<NUM_DIGITAL_PINS&&p>13)?p-14:(p>=NUM_DIGITAL_PINS)?255:p-8)
#endif

#if PIN_SPI_SS==17 // Arduino Leonardo
#define PIN_TO_PORTn_OFFSET(p) (255)
#define PIN_TO_PINn_BIT(p) (255)
#endif

#if PIN_SPI_SS==53 // Arduino Mega
#define PIN_TO_PORTn_OFFSET(p) (255)
#define PIN_TO_PINn_BIT(p) (255)
#endif

#define PIN_TO_DDRn_OFFSET(p) PIN_TO_PORTn_OFFSET(p)-1
#define PIN_TO_PINn_OFFSET(p) PIN_TO_PORTn_OFFSET(p)-2

#define _ST7735_SPI_UCSRnA_OFFSET_  0x00
#define _ST7735_SPI_UCSRnB_OFFSET_  0x01
#define _ST7735_SPI_UCSRnC_OFFSET_  0x02
#define _ST7735_SPI_UBRRnL_OFFSET_  0x04
#define _ST7735_SPI_UBRRnH_OFFSET_  0x05
#define _ST7735_SPI_UDRn_OFFSET_    0x06

#define _ST7735_SPI_UCSRnA_ADDRESS_ (ST7735_SPI_USART_SEQ_NO*8+0xC0+_ST7735_SPI_UCSRnA_OFFSET_)
#define _ST7735_SPI_UCSRnB_ADDRESS_ (_ST7735_SPI_UCSRnA_ADDRESS_+_ST7735_SPI_UCSRnB_OFFSET_)
#define _ST7735_SPI_UCSRnC_ADDRESS_ (_ST7735_SPI_UCSRnA_ADDRESS_+_ST7735_SPI_UCSRnC_OFFSET_)
#define _ST7735_SPI_UBRRnL_ADDRESS_ (_ST7735_SPI_UCSRnA_ADDRESS_+_ST7735_SPI_UBRRnL_OFFSET_)
#define _ST7735_SPI_UBRRnH_ADDRESS_ (_ST7735_SPI_UCSRnA_ADDRESS_+_ST7735_SPI_UBRRnH_OFFSET_)
#define _ST7735_SPI_UDRn_ADDRESS_   (_ST7735_SPI_UCSRnA_ADDRESS_+_ST7735_SPI_UDRn_OFFSET_)

#define _ST7735_RESET_DURATION_     10  // microseconds
#define _ST7735_AFTER_RESET_DELAY_  120 // milliseconds


#ifdef _ST7735_SPI_C_
#define _ST7735_SPI_EXTERNAL_
#define ST7735_SPI_INCLUDE_INCLUDE_FUNCTIONS
#else
#define _ST7735_SPI_EXTERNAL_ extern
#endif

typedef struct st7735_spi_buffer_status_st {
  uint8_t is_data_only          : 1; // 0 - send command (1 byte) to slave device
                                     // 1 - don't send command to slave device; continue data transfer
  uint8_t dc_bit_set            : 1; // 0 - DC bit don't set and must be handled
                                     // 1 - DC bit already set
  uint8_t reserved_2            : 1;
  uint8_t reserved_3            : 1;
  uint8_t reserved_4            : 1;
  uint8_t reserved_5            : 1;
  uint8_t reserved_6            : 1;
  uint8_t reserved_7            : 1;
} st7735_spi_buffer_status_t;

typedef struct st7735_spi_status_st {
  uint8_t  configured;
  uint8_t  chip_select_pin;
  uint8_t  reset_pin;
  uint8_t  backlight_pin;
} st7735_spi_status_t;

_ST7735_SPI_EXTERNAL_ volatile st7735_spi_status_t st7735_spi_status;

_ST7735_SPI_EXTERNAL_ void st7735_spi_configure(uint8_t ubrr_n);
_ST7735_SPI_EXTERNAL_ void st7735_spi_stop(void);
//_ST7735_SPI_EXTERNAL_ void st7735_spi_tft_init(uint8_t chip_select_pin, uint8_t reset_pin, uint8_t backlight_pin);
_ST7735_SPI_EXTERNAL_ void st7735_spi_tft_init(void);
_ST7735_SPI_EXTERNAL_ uint8_t st7735_spi_send_byte(uint8_t data, uint8_t status);
_ST7735_SPI_EXTERNAL_ void st7735_spi_send_buffer(uint8_t *data, uint8_t status, uint8_t buffer_length);
_ST7735_SPI_EXTERNAL_ void st7735_spi_send_word_data(uint16_t data);
_ST7735_SPI_EXTERNAL_ void st7735_spi_send_caset(uint8_t xs, uint8_t xe);
_ST7735_SPI_EXTERNAL_ void st7735_spi_send_raset(uint8_t ys, uint8_t ye);
_ST7735_SPI_EXTERNAL_ void st7735_spi_tft_fill_rectangle(uint8_t x, uint8_t w, uint8_t y, uint8_t h, uint16_t color);
_ST7735_SPI_EXTERNAL_ void st7735_spi_tft_write_rect_short(uint8_t x, uint8_t w, uint8_t y, uint8_t h, uint8_t *data);
_ST7735_SPI_EXTERNAL_ void st7735_spi_tft_write_rectangle(uint8_t x, uint8_t w, uint8_t y, uint8_t h, uint8_t *data);
_ST7735_SPI_EXTERNAL_ void st7735_spi_tft_read_rect_short(uint8_t x, uint8_t w, uint8_t y, uint8_t h, uint8_t *data);
_ST7735_SPI_EXTERNAL_ void st7735_spi_tft_read_rectangle(uint8_t x, uint8_t w, uint8_t y, uint8_t h, uint8_t *data);

#define st7735_spi_tft_draw_hline(x,y,l,c)  st7735_spi_tft_fill_rectangle(x, 1, y, l, c);
#define st7735_spi_tft_draw_vline(x,y,l,c)  st7735_spi_tft_fill_rectangle(x, l, y, 1, c);
#define st7735_spi_tft_draw_pixel(x,y,c)    st7735_spi_tft_fill_rectangle(x, 1, y, 1, c);
#define st7735_spi_tft_fill_screen(c)       st7735_spi_tft_fill_rectangle(x, ST7735_SCREEN_WIDTH, y, ST7735_SCREEN_HEIGHT, c);
#define st7735_spi_tft_draw_rectangle(x,y,w,h,c)    \
  st7735_spi_tft_fill_rectangle(x, w, y, 1, c);     \
  st7735_spi_tft_fill_rectangle(x+h-1, w, y, 1, c); \
  st7735_spi_tft_fill_rectangle(x, 1, y, h, c);     \
  st7735_spi_tft_fill_rectangle(x, 1, y+l-1, h, c);


#define _ST7735_SPI_COMMAND_SLPOUT_CODE_  0x11
#define _ST7735_SPI_COMMAND_COLMOD_CODE_  0x3A
#define _ST7735_SPI_COMMAND_COLMOD_DATA_  0x55
#define _ST7735_SPI_COMMAND_COLMOD_MAX_   0x66
#define _ST7735_SPI_COMMAND_DISPON_CODE_  0x29
#define _ST7735_SPI_COMMAND_CASET_CODE_   0x2A
#define _ST7735_SPI_COMMAND_RASET_CODE_   0x2B
#define _ST7735_SPI_COMMAND_RAMWR_CODE_   0x2C
#define _ST7735_SPI_COMMAND_RAMRD_CODE_   0x2E

#define _ST7735_SPI_RAMRD_DELAY_CYCLES_   (350/(1000/(F_CPU/1000000))/3+1)

#define ST7735_BLACK        0x0000
#define ST7735_WHITE        0xFFFF
#define ST7735_RED          0xF800
#define ST7735_GREEN        0x07E0
#define ST7735_BLUE         0x001F
#define ST7735_NAVY         0x000F
#define ST7735_DARKGREEN    0x03E0
#define ST7735_DARKCYAN     0x03EF
#define ST7735_MAROON       0x7800
#define ST7735_PURPLE       0x780F
#define ST7735_OLIVE        0x7BE0
#define ST7735_LIGHTGREY    0xC618
#define ST7735_DARKGREY     0x7BEF
#define ST7735_CYAN         0x07FF
#define ST7735_MAGENTA      0xF81F
#define ST7735_YELLOW       0xFFE0
#define ST7735_ORANGE       0xFD20
#define ST7735_GREENYELLOW  0xAFE5
#define ST7735_PINK         0xF81F

#ifdef __cplusplus
}
#endif

#endif // _ST7735_SPI_H_INCLUDED

