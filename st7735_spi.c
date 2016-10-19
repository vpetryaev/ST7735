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

#define _ST7735_SPI_C_

#define ST7735_RESET_PIN       8
#define ST7735_BACKLIGHT_PIN   7

#include "st7735_spi.h"

#if ST7735_RESET_PIN==255 && ST7735_BACKLIGHT_PIN==255
void st7735_spi_tft_init(uint8_t chip_select_pin, uint8_t reset_pin, uint8_t backlight_pin)
{
  if ( !(reset_pin+1) )
  {
    _SFR_IO8(PIN_TO_DDRn_OFFSET(reset_pin))  |=  (1<<PIN_TO_PINn_BIT(reset_pin));
    _SFR_IO8(PIN_TO_PORTn_OFFSET(reset_pin)) &= ~(1<<PIN_TO_PINn_BIT(reset_pin));
    delayMicroseconds(_ST7735_RESET_DURATION_);
    _SFR_IO8(PIN_TO_PORTn_OFFSET(reset_pin)) |=  (1<<PIN_TO_PINn_BIT(reset_pin));
  }
  if ( !(backlight_pin+1) )
  {
    _SFR_IO8(PIN_TO_DDRn_OFFSET(backlight_pin))  |=  (1<<PIN_TO_PINn_BIT(backlight_pin));
    _SFR_IO8(PIN_TO_PORTn_OFFSET(backlight_pin)) |=  (1<<PIN_TO_PINn_BIT(backlight_pin));
  }
#else
void st7735_spi_tft_init(void)
{
#if ST7735_RESET_PIN!=255
  _SFR_IO8(PIN_TO_DDRn_OFFSET(ST7735_RESET_PIN))  |=  (1<<PIN_TO_PINn_BIT(ST7735_RESET_PIN));
  _SFR_IO8(PIN_TO_PORTn_OFFSET(ST7735_RESET_PIN)) &= ~(1<<PIN_TO_PINn_BIT(ST7735_RESET_PIN));
  delayMicroseconds(_ST7735_RESET_DURATION_);
  _SFR_IO8(PIN_TO_PORTn_OFFSET(ST7735_RESET_PIN)) |=  (1<<PIN_TO_PINn_BIT(ST7735_RESET_PIN));
#endif
#if ST7735_BACKLIGHT_PIN!=255
  _SFR_IO8(PIN_TO_DDRn_OFFSET(ST7735_BACKLIGHT_PIN))  |=  (1<<PIN_TO_PINn_BIT(ST7735_BACKLIGHT_PIN));
  _SFR_IO8(PIN_TO_PORTn_OFFSET(ST7735_BACKLIGHT_PIN)) |=  (1<<PIN_TO_PINn_BIT(ST7735_BACKLIGHT_PIN));
#endif
#endif
  _SFR_IO8(PIN_TO_DDRn_OFFSET(ST7735_CHIPSELECT_PIN))  |=  (1<<PIN_TO_PINn_BIT(ST7735_CHIPSELECT_PIN));
  _SFR_IO8(PIN_TO_PORTn_OFFSET(ST7735_CHIPSELECT_PIN)) |=  (1<<PIN_TO_PINn_BIT(ST7735_CHIPSELECT_PIN));
  delay(_ST7735_AFTER_RESET_DELAY_);
  st7735_spi_configure(0);
  _SFR_IO8(PIN_TO_PORTn_OFFSET(ST7735_CHIPSELECT_PIN)) &= ~(1<<PIN_TO_PINn_BIT(ST7735_CHIPSELECT_PIN));
  st7735_spi_send_byte(_ST7735_SPI_COMMAND_SLPOUT_CODE_, 0);
  delay(_ST7735_AFTER_RESET_DELAY_);
  st7735_spi_send_byte(_ST7735_SPI_COMMAND_COLMOD_CODE_, 0);
  st7735_spi_send_byte(_ST7735_SPI_COMMAND_COLMOD_DATA_, 1);
  st7735_spi_send_byte(_ST7735_SPI_COMMAND_DISPON_CODE_, 0);
  return;
}

/*-----------------13.09.2016 22:56-----------------
 * st7735_spi_configure function:
 * - configure USART for master SPI mode
 * - one parameter is Fosc divider
 * - allow set baudrate from Fosc/256 to Fosc/2 or
 *   from 32.25Kbaud to 8Mbaud for 16MHz Fosc
 * --------------------------------------------------*/

void st7735_spi_configure(uint8_t ubrr_n)
{
  if ( st7735_spi_status.configured )
  {
    st7735_spi_stop();
  }
  asm volatile ( ";                               \n\t \
  clr   r25                                       \n\t \
  std   %a[UCSRnA_addr]+%[UBRRnH_offs], r25       \n\t \
  std   %a[UCSRnA_addr]+%[UBRRnL_offs], r25       \n\t \
  sbi   %[xck_ddr_offs],%[xck_pin]                \n\t \
  ldi   r25,%[UCSRnC_value]                       \n\t \
  std   %a[UCSRnA_addr]+%[UCSRnC_offs], r25       \n\t \
  ldi   r25,%[UCSRnB_value]                       \n\t \
  std   %a[UCSRnA_addr]+%[UCSRnB_offs], r25       \n\t \
  std   %a[UCSRnA_addr]+%[UBRRnL_offs], %[ubrr_n] \n\t \
  cbi   %[dc_port_offset],%[dc_pin]               \n\t \
  sbi   %[dc_ddr_offs],%[dc_pin]                  \n\t \
  "
  :
  : [UCSRnA_addr] "b" (_ST7735_SPI_UCSRnA_ADDRESS_),
    [UCSRnB_offs] "I" (_ST7735_SPI_UCSRnB_OFFSET_),
    [UCSRnB_value] "M" ((0<<RXCIE0)|(0<<TXCIE0)|(0<<UDRIE0)|(0<<RXEN0)|(1<<TXEN0)),
    [UCSRnC_offs] "I" (_ST7735_SPI_UCSRnC_OFFSET_),
    [UCSRnC_value] "M" ((0<<UCPOL0)|(0<<UCPHA0)|(0<<UDORD0)|(1<<UMSEL00)|(1<<UMSEL01)),
    [UBRRnL_offs] "I" (_ST7735_SPI_UBRRnL_OFFSET_),
    [UBRRnH_offs] "I" (_ST7735_SPI_UBRRnH_OFFSET_),
    [xck_ddr_offs] "I" (PIN_TO_DDRn_OFFSET(PIN_MSPIM_XCK)),
    [xck_pin] "I" (PIN_TO_PINn_BIT(PIN_MSPIM_XCK)),
    [dc_ddr_offs] "I" (PIN_TO_DDRn_OFFSET(ST7735_SPI_DC_PIN_NO)),
    [dc_port_offset] "I" (PIN_TO_PORTn_OFFSET(ST7735_SPI_DC_PIN_NO)),
    [dc_pin] "I" (PIN_TO_PINn_BIT(ST7735_SPI_DC_PIN_NO)),
    [ubrr_n] "r" (ubrr_n)
  : "r25", "cc" );
  st7735_spi_status.configured=1;
}

void st7735_spi_stop(void)
{
  uint8_t ubrr_n = _SFR_MEM8(_ST7735_SPI_UBRRnL_OFFSET_);

  if ( !st7735_spi_status.configured )
  {
    return;
  }
  while ( !(_SFR_MEM8(_ST7735_SPI_UCSRnA_ADDRESS_)&(1<<UDRE0)) );
  delayMicroseconds((uint16_t)ubrr_n+1);
  _SFR_IO8(_ST7735_SPI_UBRRnL_OFFSET_)=0;
  _SFR_IO8(_ST7735_SPI_UCSRnB_OFFSET_)=0;
  _SFR_IO8(_ST7735_SPI_UCSRnC_OFFSET_)=0;
  _SFR_IO8(PIN_TO_PORTn_OFFSET(ST7735_SPI_DC_PIN_NO))&= ~(1<<PIN_TO_PINn_BIT(ST7735_SPI_DC_PIN_NO));
  _SFR_IO8(PIN_TO_DDRn_OFFSET(ST7735_SPI_DC_PIN_NO))&= ~(1<<PIN_TO_PINn_BIT(ST7735_SPI_DC_PIN_NO));
  _SFR_IO8(PIN_TO_PORTn_OFFSET(ST7735_CHIPSELECT_PIN)) |=  (1<<PIN_TO_PINn_BIT(ST7735_CHIPSELECT_PIN));
  return;
}

uint8_t st7735_spi_send_byte(uint8_t data, uint8_t status)
{
  asm volatile ( ";                             \n   \
WAIT_UDREn_ON_START%=:                          \n\t \
  lds   __tmp_reg__, %[UCSRnA_addr]   ;2        \n\t \
  sbrs  __tmp_reg__, %[UDREn_bit]     ;1/2      \n\t \
  rjmp  WAIT_UDREn_ON_START%=         ;2        \n\t \
  cli                                 ;1        \n\t \
  sts   %[UDRn_addr], %[data]         ;2   (7)  \n\t \
  sbrc  %[status], 1                  ;1/2      \n\t \
  rjmp  EXIT_ASM%=                    ;2   (10) \n\t \
  sbrc  %[status], 0                  ;1/2      \n\t \
  rjmp  SET_DC_FOR_DATA%=             ;2   (12) \n   \
; Set DC for command                            \n\t \
  sbis  %[port],%[pin]                ;1/2      \n\t \
  rjmp  EXIT_ASM%=                    ;2   (14) \n   \
WAIT_UDREn_FOR_COMMAND%=:                       \n\t \
  lds   __tmp_reg__, %[UCSRnA_addr]   ;2        \n\t \
  sbrs  __tmp_reg__, %[UDREn_bit]     ;1/2      \n\t \
  rjmp  WAIT_UDREn_FOR_COMMAND%=      ;2        \n\t \
  cbi   %[port],%[pin]                ;2        \n\t \
  rjmp  EXIT_ASM%=                    ;2   (21) \n   \
SET_DC_FOR_DATA%=:                              \n\t \
  sbic  %[port],%[pin]                ;1/2      \n\t \
  rjmp  SET_DC_STATUS%=               ;2   (15) \n   \
WAIT_UDREn_FOR_DATA%=:                          \n\t \
  lds   __tmp_reg__, %[UCSRnA_addr]   ;2        \n\t \
  sbrs  __tmp_reg__, %[UDREn_bit]     ;1/2      \n\t \
  rjmp  WAIT_UDREn_FOR_DATA%=         ;2        \n\t \
  sbi   %[port],%[pin]                ;2   (20) \n   \
SET_DC_STATUS%=:                                \n\t \
  sbr   %[status], 1                  ;1        \n\t \
EXIT_ASM%=:                                     \n\t \
  sei                                 ;1        \n\t \
  "
  : [status] "+r" (status)
  : [data] "r" (data),
    [UCSRnA_addr] "M" (_ST7735_SPI_UCSRnA_ADDRESS_),
    [UDREn_bit] "I" (UDRE0),
    [UDRn_addr] "M" (_ST7735_SPI_UDRn_ADDRESS_),
    [port] "I" (PIN_TO_PORTn_OFFSET(ST7735_SPI_DC_PIN_NO)),
    [pin] "I" (PIN_TO_PINn_BIT(ST7735_SPI_DC_PIN_NO))
  : "cc" );
  return(status);
}

/*-----------------12.09.2016 13:17-----------------
 * Buffer send function:
 * - status not updated
 * - status have bits according to st7735_spi_buffer_status_t
 * - function don't check dc_bit_set bit in status
 * - send command with data => status = 0
 * - send only data => status=1 or is_data_only=1
 * - buffer_length=0 allow to send 256 bytes in buffer
 * - function don't check buffer pointer; be carefull!
 * --------------------------------------------------*/

void st7735_spi_send_buffer(uint8_t *data, uint8_t status, uint8_t buffer_length)
{
  uint8_t readb;

  asm volatile ( ";                             \n\t \
  ld    %[readb], %a[data]+           ;2        \n   \
WAIT_UDREn_ON_START%=:                          \n\t \
  lds   __tmp_reg__, %[UCSRnA_addr]   ;2        \n\t \
  sbrs  __tmp_reg__, %[UDREn_bit]     ;1/2      \n\t \
  rjmp  WAIT_UDREn_ON_START%=         ;2        \n\t \
  cli                                 ;1        \n\t \
  sts   %[UDRn_addr], %[readb]        ;2        \n\t \
  ld    %[readb], %a[data]+           ;2        \n\t \
  sbrc  %[status], 0                  ;1/2      \n\t \
  rjmp  WAIT_UDREn_FOR_DATA%=         ;2        \n   \
; Set DC for command                            \n   \
WAIT_UDREn_FOR_COMMAND%=:                       \n\t \
  lds   __tmp_reg__, %[UCSRnA_addr]   ;2        \n\t \
  sbrs  __tmp_reg__, %[UDREn_bit]     ;1/2      \n\t \
  rjmp  WAIT_UDREn_FOR_COMMAND%=      ;2        \n\t \
  cbi   %[port],%[pin]                ;2        \n\t \
  dec   %[buffer_length]              ;1        \n\t \
  BREQ  EXIT_ASM%=                    ;1        \n\t \
  ld    %[status], %a[data]+          ;2        \n\t \
  sts   %[UDRn_addr], %[readb]        ;2   (16) \n\t \
  mov   %[readb], %[status]           ;1        \n   \
WAIT_UDREn_FOR_DATA%=:                          \n\t \
  lds   __tmp_reg__, %[UCSRnA_addr]   ;2        \n\t \
  sbrs  __tmp_reg__, %[UDREn_bit]     ;1/2      \n\t \
  rjmp  WAIT_UDREn_FOR_DATA%=         ;2        \n\t \
  sbi   %[port],%[pin]                ;2        \n\t \
  sei                                 ;1   (8)  \n   \
DATA_LOOP%=:                                    \n\t \
  dec   %[buffer_length]              ;1        \n\t \
  BREQ  EXIT_ASM%=                    ;1        \n   \
WAIT_UDREn_IN_DATA_LOOP%=:                      \n\t \
  lds   __tmp_reg__, %[UCSRnA_addr]   ;2        \n\t \
  sbrs  __tmp_reg__, %[UDREn_bit]     ;1/2      \n\t \
  rjmp  WAIT_UDREn_IN_DATA_LOOP%=     ;2   (5)  \n\t \
  sts   %[UDRn_addr], %[readb]        ;2   (16) \n\t \
  ld    %[readb], %a[data]+           ;2        \n\t \
  rjmp  DATA_LOOP%=                   ;2   (12) \n   \
EXIT_ASM%= :                                    \n\t \
  sei                                 ;1        \n\t \
  "
  : [data] "+b" (data), [status] "+d" (status), [buffer_length] "+r" (buffer_length),
    [readb] "=r" (readb)
  : [UCSRnA_addr] "M" (_ST7735_SPI_UCSRnA_ADDRESS_),
    [UDREn_bit] "I" (UDRE0), [UDRn_addr] "M" (_ST7735_SPI_UDRn_ADDRESS_),
    [port] "I" (PIN_TO_PORTn_OFFSET(ST7735_SPI_DC_PIN_NO)),
    [pin] "I" (PIN_TO_PINn_BIT(ST7735_SPI_DC_PIN_NO))
  : "cc" );
}

void st7735_spi_send_word_data(uint16_t data)
{
  asm volatile ( ";                             \n   \
WAIT_UDREn_ON_START%=:                          \n\t \
  lds   __tmp_reg__, %[UCSRnA_addr]   ;2        \n\t \
  sbrs  __tmp_reg__, %[UDREn_bit]     ;1/2      \n\t \
  rjmp  WAIT_UDREn_ON_START%=         ;2        \n\t \
  cli                                 ;1        \n\t \
  sts   %[UDRn_addr], r27             ;2   (7)  \n\t \
WAIT_UDREn_FOR_DATA%=:                          \n\t \
  lds   __tmp_reg__, %[UCSRnA_addr]   ;2        \n\t \
  sbrs  __tmp_reg__, %[UDREn_bit]     ;1/2      \n\t \
  rjmp  WAIT_UDREn_FOR_DATA%=         ;2        \n\t \
  sbi   %[port],%[pin]                ;2   (6)  \n   \
  sei                                 ;1        \n\t \
  sts   %[UDRn_addr], r26             ;2   (9)  \n\t \
  "
  :
  : [data] "x" (data),
    [UCSRnA_addr] "M" (_ST7735_SPI_UCSRnA_ADDRESS_),
    [UDREn_bit] "I" (UDRE0),
    [UDRn_addr] "M" (_ST7735_SPI_UDRn_ADDRESS_),
    [port] "I" (PIN_TO_PORTn_OFFSET(ST7735_SPI_DC_PIN_NO)),
    [pin] "I" (PIN_TO_PINn_BIT(ST7735_SPI_DC_PIN_NO))
  : "cc" );
}

void st7735_spi_read_frame_memory(uint8_t *data, uint8_t status, uint8_t buffer_length)
{
  uint8_t firstb, readb, lastb, filler;

  asm volatile ( ";                             \n\t \
  clr   %[filler]                     ;1        \n\t \
  dec   %[filler]                     ;1        \n\t \
  cpi   %[status], 0                  ;1        \n\t \
  breq  FLUSH_RECEIVER%=              ;1/2      \n\t \
  clr   %[status]                     ;1        \n   \
; Rx = data, RxBuf = empty, RxEDR = data        \n   \
WAIT_UDREn_FOR_DATA%=:                          \n\t \
  lds   __tmp_reg__, %[UCSRnA_addr]   ;2        \n\t \
  sbrs  __tmp_reg__, %[UDREn_bit]     ;1/2      \n\t \
  rjmp  WAIT_UDREn_FOR_DATA%=         ;2        \n   \
  sts   %[UDRn_addr], %[filler]       ;2   (8)  \n   \
; Rx = data, RxBuf = data, RxEDR = data         \n\t \
WAIT_RXCn_FOR_DATA%=:                           \n\t \
  lds   __tmp_reg__, %[UCSRnA_addr]   ;2        \n\t \
  sbrs  __tmp_reg__, %[RXCn_bit]      ;1/2      \n\t \
  rjmp  WAIT_RXCn_FOR_DATA%=          ;2        \n\t \
  lds   %[readb], %[UDRn_addr]        ;2        \n   \
; Rx = data, RxBuf = empty, RxEDR = data        \n\t \
  cpi   %[status], 0                  ;1   (9)  \n\t \
  brlt  COLOR_LAST_BYTE%=             ;1/2      \n\t \
  breq  COLOR_FIRST_BYTE%=            ;1/2      \n\t \
  swap  %[readb]                      ;1        \n\t \
  mov   %[lastb], %[readb]            ;1        \n\t \
  andi  %[readb], 0b00000111          ;1        \n\t \
  or    %[readb], %[firstb]           ;1        \n\t \
  ldi   %[status], 0xFF               ;1        \n\t \
  rjmp  CONTINUE_LOOP%=               ;2   (20) \n   \
COLOR_FIRST_BYTE%=:                             \n\t \
  lsl   %[readb]                      ;1        \n\t \
  andi  %[readb], 0b11111000          ;1        \n\t \
  mov   %[firstb], %[readb]           ;1        \n\t \
  inc   %[status]                     ;1        \n\t \
  rjmp  WAIT_UDREn_FOR_DATA%=         ;2   (17) \n   \
COLOR_LAST_BYTE%=:                              \n\t \
  lsr   %[readb]                      ;1        \n\t \
  lsr   %[readb]                      ;1        \n\t \
  andi  %[lastb], 0b11100000          ;1        \n\t \
  or    %[readb], %[lastb]            ;1        \n\t \
  clr   %[status]                     ;1        \n   \
CONTINUE_LOOP%=:                                \n\t \
  st    %a[data]+, %[readb]           ;2        \n\t \
  dec   %[buffer_length]              ;1        \n\t \
  brne  WAIT_UDREn_FOR_DATA%=         ;1/2      \n\t \
  rjmp  EXIT_ASM%=                    ;2        \n   \
FLUSH_RECEIVER%=:                               \n\t \
  lds   __tmp_reg__, %[UCSRnA_addr]   ;2        \n\t \
  sbrs  __tmp_reg__, %[UDREn_bit]     ;1/2      \n\t \
  rjmp  FLUSH_RECEIVER%=              ;2        \n\t \
  sts   %[UCSRnA_addr], __zero_reg__  ;2        \n\t \
  ldi   %[readb], %[rdmemcmd]         ;1        \n\t \
  cli                                 ;1        \n\t \
  sts   %[UDRn_addr], %[readb]        ;2        \n   \
; Tx = any, TxBuf = rdmemcmd                    \n   \
; Rx disabled                                   \n   \
WAIT_UDREn_FOR_COMMAND%=:                       \n\t \
  lds   __tmp_reg__, %[UCSRnA_addr]   ;2        \n\t \
  sbrs  __tmp_reg__, %[UDREn_bit]     ;1/2      \n\t \
  rjmp  WAIT_UDREn_FOR_COMMAND%=      ;2        \n\t \
  cbi   %[dc_port], %[dc_pin]         ;2        \n\t \
  ldi   %[lastb], %[TXCn_clear]       ;1        \n   \
WAIT_UDREn_BEFORE_DUMMY%=:                      \n\t \
  lds   __tmp_reg__, %[UCSRnA_addr]   ;2        \n\t \
  sbrs  __tmp_reg__, %[UDREn_bit]     ;1/2      \n\t \
  rjmp  WAIT_UDREn_BEFORE_DUMMY%=     ;2        \n\t \
  sts   %[UDRn_addr], %[filler]       ;2        \n\t \
  sts   %[UCSRnA_addr], %[lastb]      ;2        \n   \
; Tx = rdmemcmd, TxBuf = dummy                  \n   \
; Rx disabled                                   \n   \
WAIT_UDREn_FOR_DUMMY%=:                         \n\t \
  lds   __tmp_reg__, %[UCSRnA_addr]   ;2        \n\t \
  sbrs  __tmp_reg__, %[UDREn_bit]     ;1/2      \n\t \
  rjmp  WAIT_UDREn_FOR_DUMMY%=        ;2        \n   \
; Tx = dummy, TxBuf = empty                     \n   \
; Rx disabled                                   \n   \
  sbi   %[dc_port],%[dc_pin]          ;2        \n\t \
  sei                                 ;1        \n\t \
  ldi   %[status], %[RX_Enable]       ;1        \n\t \
WAIT_TXCn_FOR_DUMMY%=:                          \n\t \
  lds   __tmp_reg__, %[UCSRnA_addr]   ;2        \n\t \
  sbrs  __tmp_reg__, %[TXCn_bit]      ;1/2      \n\t \
  rjmp  WAIT_TXCn_FOR_DUMMY%=         ;2        \n\t \
  sts   %[UCSRnA_addr], %[lastb]      ;2        \n\t \
; Tx = dummy, TxBuf = dummy                     \n   \
; Rx disabled                                   \n\t \
  sts   %[UCSRnB_addr], %[status]     ;2        \n\t \
  clr   %[status]                     ;1        \n   \
  rjmp  WAIT_UDREn_FOR_DATA%=         ;2        \n   \
EXIT_ASM%=:                                     \n   \
  "
  : [data] "+b" (data), [status] "+d" (status), [buffer_length] "+r" (buffer_length),
    [firstb] "=r" (firstb), [readb] "=d" (readb), [lastb] "=d" (lastb),
    [filler] "=r" (filler)
  : [rdmemcmd] "I" (_ST7735_SPI_COMMAND_RAMRD_CODE_),
    [UCSRnA_addr] "M" (_ST7735_SPI_UCSRnA_ADDRESS_),
    [UDREn_bit] "I" (UDRE0),
    [UCSRnB_addr] "M" (_ST7735_SPI_UCSRnB_ADDRESS_),
    [RX_Enable] "M" ((0<<RXCIE0)|(0<<TXCIE0)|(0<<UDRIE0)|(1<<RXEN0)|(1<<TXEN0)),
    [RX_Disable] "M" ((0<<RXCIE0)|(0<<TXCIE0)|(0<<UDRIE0)|(0<<RXEN0)|(1<<TXEN0)),
    [TXCn_bit] "I" (TXC0),
    [RXCn_bit] "I" (RXC0),
    [TXCn_clear] "M" (1<<TXC0),
    [UDRn_addr] "M" (_ST7735_SPI_UDRn_ADDRESS_),
    [dc_port] "I" (PIN_TO_PORTn_OFFSET(ST7735_SPI_DC_PIN_NO)),
    [dc_pin] "I" (PIN_TO_PINn_BIT(ST7735_SPI_DC_PIN_NO))
  : "cc" );
  return;
}


void st7735_spi_send_caset(uint8_t xs, uint8_t xe)
{
  st7735_spi_send_byte(_ST7735_SPI_COMMAND_CASET_CODE_, 0);
  st7735_spi_send_word_data(xs);
  st7735_spi_send_word_data(xe);
}

void st7735_spi_send_raset(uint8_t ys, uint8_t ye)
{
  st7735_spi_send_byte(_ST7735_SPI_COMMAND_RASET_CODE_, 0);
  st7735_spi_send_word_data(ys);
  st7735_spi_send_word_data(ye);
}

void st7735_spi_tft_fill_rectangle(uint8_t x, uint8_t w, uint8_t y, uint8_t h, uint16_t color)
{
  if ( !(w||h))
  {
    return;
  }
  st7735_spi_send_caset(x, x+w-1);
  st7735_spi_send_raset(y, y+h-1);
  st7735_spi_send_byte(_ST7735_SPI_COMMAND_RAMWR_CODE_, 0);
  for ( uint8_t i=0; i<w; i++ )
  {
    for ( uint8_t j=0; j<h; j++ )
    {
      st7735_spi_send_word_data(color);
    }
  }
}

void st7735_spi_tft_write_rect_short(uint8_t x, uint8_t w, uint8_t y, uint8_t h, uint8_t *data)
{
  st7735_spi_send_caset(x, x+w-1);
  if ( (uint16_t)w*(uint16_t)h>127 )
  {
    w=0;
  }
  st7735_spi_send_raset(y, y+h-1);
  st7735_spi_send_byte(_ST7735_SPI_COMMAND_RAMWR_CODE_, 0);
  st7735_spi_send_buffer(data, 1, w*h*2);
}

void st7735_spi_tft_write_rectangle(uint8_t x, uint8_t w, uint8_t y, uint8_t h, uint8_t *data)
{
  uint8_t i, len;
  uint16_t buffer_length;

  if ( !(w||h))
  {
    return;
  }
  st7735_spi_send_caset(x, x+w-1);
  buffer_length=(uint16_t)w*(uint16_t)h*2;
  len=(uint8_t)(buffer_length&0xFF);
  st7735_spi_send_raset(y, y+h-1);
  for ( i=(uint8_t)(buffer_length>>8); i>0 || (i && len); i-- )
  {
    st7735_spi_send_buffer(data, 1, (i)?0:len);
    data+=256;
  }
}

static inline void st7735_spi_tft_read_cs_strobe(void)
{
  uint8_t counter, status;

  asm volatile ( ";                 \n\t \
  ldi   %[status],%[TX_Disable]     \n\t \
  sts   %[UCSRnB_addr], %[status]   \n\t \
  sbi   %[cs_port],%[cs_pin]        \n\t \
  ldi   %[counter], %[delay]  ;1    \n   \
WAIT_LOOP%=:                        \n\t \
  dec   %[counter]            ;1    \n\t \
  brne  WAIT_LOOP%=           ;1/2  \n\t \
  cbi   %[cs_port],%[cs_pin]  ;2    \n\t \
  ldi   %[status],%[TX_Enable]      \n\t \
  sts   %[UCSRnB_addr], %[status]   \n   \
  "
  : [counter] "=d" (counter), [status] "=d" (status)
  : [delay] "I" (_ST7735_SPI_RAMRD_DELAY_CYCLES_/3+1),
    [UCSRnB_addr] "M" (_ST7735_SPI_UCSRnB_ADDRESS_),
    [TX_Enable] "M" ((0<<RXCIE0)|(0<<TXCIE0)|(0<<UDRIE0)|(0<<RXEN0)|(1<<TXEN0)),
    [TX_Disable] "M" ((0<<RXCIE0)|(0<<TXCIE0)|(0<<UDRIE0)|(0<<RXEN0)|(0<<TXEN0)),
    [cs_port] "I" (PIN_TO_PORTn_OFFSET(ST7735_CHIPSELECT_PIN)),
    [cs_pin] "I" (PIN_TO_PINn_BIT(ST7735_CHIPSELECT_PIN))
  : "cc" );
}

void st7735_spi_tft_read_rect_short(uint8_t x, uint8_t w, uint8_t y, uint8_t h, uint8_t *data)
{
  if ( !(w||h))
  {
    return;
  }
  st7735_spi_send_byte(_ST7735_SPI_COMMAND_COLMOD_CODE_, 0);
  st7735_spi_send_byte(_ST7735_SPI_COMMAND_COLMOD_MAX_, 1);
  st7735_spi_send_caset(x, x+w-1);
  if ( (uint16_t)w*(uint16_t)h>127 )
  {
    w=0;
  }
  st7735_spi_send_raset(y, y+h-1);
  st7735_spi_read_frame_memory(data, 0, w*h*2);
  st7735_spi_tft_read_cs_strobe();
  st7735_spi_send_byte(_ST7735_SPI_COMMAND_COLMOD_CODE_, 0);
  st7735_spi_send_byte(_ST7735_SPI_COMMAND_COLMOD_DATA_, 1);
}

void st7735_spi_tft_read_rectangle(uint8_t x, uint8_t w, uint8_t y, uint8_t h, uint8_t *data)
{
  uint8_t i, len, status=0;
  uint16_t buffer_length;

  if ( !(w||h))
  {
    return;
  }
  st7735_spi_send_byte(_ST7735_SPI_COMMAND_COLMOD_CODE_, 0);
  st7735_spi_send_byte(_ST7735_SPI_COMMAND_COLMOD_MAX_, 1);
  st7735_spi_send_caset(x, x+w-1);
  buffer_length=(uint16_t)w*(uint16_t)h*2;
  len=(uint8_t)(buffer_length&0xFF);
  st7735_spi_send_raset(y, y+h-1);
  for ( i=(uint8_t)(buffer_length>>8); i>0 || (i && len); i-- )
  {
    st7735_spi_read_frame_memory(data, status, (i)?0:len);
    status=1;
    data+=256;
  }
  st7735_spi_tft_read_cs_strobe();
  st7735_spi_send_byte(_ST7735_SPI_COMMAND_COLMOD_CODE_, 0);
  st7735_spi_send_byte(_ST7735_SPI_COMMAND_COLMOD_DATA_, 1);
}

#undef _ST7735_SPI_C_
