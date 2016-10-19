/*
 * AVRkanoid.h
 *
 *  Created on: 15 oct. 2016 ã.
 *      Author: ptr
 */

#ifndef _AVRKANOID_H_INCLUDED
#define _AVRKANOID_H_INCLUDED

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "st7735_sprite.h"

#ifdef _AVRKANOID_C_
#define _AVRKANOID_EXTERNAL_
#else
#define _AVRKANOID_EXTERNAL_ extern
#endif

#define AVRKANOID_RIGHT_PIN     PIN_A0
#define AVRKANOID_LEFT_PIN     	PIN_A1

#define AVRKANOID_BALL_SIZE			5  // Max ball size is 8
#define AVRKANOID_BALL_ALPHA  	"\x88\x00\x08\x80"
#define AVRKANOID_BALL_COLOR		ST7735_GREEN
#define AVRKANOID_BALL_BORDER		ST7735_DARKGREEN

#define AVRKANOID_RACKET_HEIGHT	4 // Max racket size calculated as Height*Width<=128
#define AVRKANOID_RACKET_WIDTH 	16
#define AVRKANOID_RACKET_ALPHA  "\x80\x01\x00\x00\x00\x00\x80\x01"
#define AVRKANOID_RACKET_COLOR 	ST7735_YELLOW
#define AVRKANOID_RACKET_BORDER	ST7735_OLIVE
#define AVRKANOID_RACKET_FROM_Y 2

#define AVRKANOID_BRICK_COLOR		ST7735_RED
#define AVRKANOID_BRICK_BORDER	ST7735_PURPLE
#define AVRKANOID_BRICK_HEIGHT	6
#define AVRKANOID_BRICK_WIDTH 	16  // Important! ST7735_SCREEN_WIDTH%AVRKANOID_BRICK_WIDTH must be zero!
#define AVRKANOID_BRICK_LAYERS 	8
#define AVRKANOID_BRICK_FROM_Y 	8
#define AVRKANOID_BRICKS_MASK0	"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF"

#define AVRKANOID_BACK_COLOR_START	0x0002
#define AVRKANOID_BACK_COLOR_STEP		7

#define AVRKANOID_DELAY_SCALER			8
#define AVRKANOID_FRAME_DELAY 			192
#define AVRKANOID_FRAME_DELAY_STEP	4
#define AVRKANOID_RACKET_DELAY			32
#define AVRKANOID_GAME_FINISH_DELAY	3000

typedef struct avrkanoid_data_st {
	sprite_t *ball_sprite;
	sprite_t *racket_sprite;
	uint8_t  *bricks_bitmask;
	uint8_t frame_delay;
	uint8_t frame_delay_step;
	uint8_t racket_delay;
} avrkanoid_data_t;

_AVRKANOID_EXTERNAL_ uint8_t AVRkanoid_setup(void);
_AVRKANOID_EXTERNAL_ uint8_t AVRkanoid_new_game(void);
_AVRKANOID_EXTERNAL_ void AVRkanoid_loop(void);

_AVRKANOID_EXTERNAL_ avrkanoid_data_t *avrkanoid_data;

#define AVRKANOID_ERR_NOMEMORY	64

#ifdef __cplusplus
}
#endif

#endif // _AVRKANOID_H_INCLUDED
