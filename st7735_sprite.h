#ifndef _ST7735_SPRITE_H_INCLUDED
#define _ST7735_SPRITE_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "st7735_spi.h"

#ifdef _ST7735_SPRITE_C_
#define _ST7735_SPRITE_EXTERNAL_
#else
#define _ST7735_SPRITE_EXTERNAL_ extern
#endif


/*-----------------08.10.2016 11:08-----------------
 * Sprite direction linear. Defined as vector from 0 to point.
 * One of point coordinate always 7 by moduele,
 * another may varying from 0 to 7 by module.
 * Defined 4 sectors:
 *   x>0 and y>0 then direction_right=1 and direction_up=1
 *   x>0 and y<0 - direction_right=1 and direction_up=0
 *   x<0 and y<0 - direction_right=0 and direction_up=0
 *   x<0 and y>0 - direction_right=0 and direction_up=1
 * In each sector defined 16 direction for |x|=15
 * and 16 direction for |y|=15
 * Direction |x|=|y|=15 common. Total 31 directions in sector.
 * Directions with |x|=0 or |y|=0 identical for sectors pair.
 * Total 120 directions. ~3 degree each +-1 degree
 * (near 0X and 0Y ~ 3.8 degree and ~2 degree near diagonals
 * --------------------------------------------------*/
#define SPRITE_MAX_SHIFT		15
#define SPRITE_CALC_SCALER	16

typedef struct sprite_st {
  uint16_t *sprite;
  uint8_t width;
  uint8_t height;
  uint8_t *alfa_channel_bitmask;  // alpha channel mask
  uint8_t x;
  uint8_t y;
  uint8_t direction_stopped:1;    // 0 - move sprite, 1 - stop sprite
  uint8_t direction_right:1;      // 0 - left, 1 - right
  uint8_t direction_up:1;         // 0 - down, 1 - up
  uint8_t direction_vert:1;       // 0 - horizontal, 1 - vertical
  uint8_t direction_shift:4;      // shift from vertical or horizontal according to direction_vert
  uint8_t direction_cur_x:4;      // for horizontal direction 15, for vertical is direction_shift; decrement to 0
  uint8_t direction_cur_y:4;      // for vertical direction 15, for horizontal is direction_shift; decrement to 0
  uint16_t *filled;
} sprite_t;

_ST7735_SPRITE_EXTERNAL_ void draw_sprite(sprite_t *sprite);
_ST7735_SPRITE_EXTERNAL_ uint8_t move_sprite(sprite_t *sprite);

#ifdef __cplusplus
}
#endif

#endif // _ST7735_SPRITE_H_INCLUDED
