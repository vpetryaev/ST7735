#define _ST7735_SPRITE_C_
#include "st7735_sprite.h"

static void apply_alpha(sprite_t *sprite)
{
  uint8_t i, m, mask;

  if ( !sprite->alfa_channel_bitmask ) return;
  for ( i=0, m=0, mask=128; i<(sprite->width*sprite->height); i++ )
  {
    if ( sprite->alfa_channel_bitmask[m]&mask )
    {
      sprite->sprite[i]=sprite->filled[i];
    }
    mask>>=1;
    if ( !mask )
    {
      mask=128;
      m++;
    }
  }
  return;
}

void draw_sprite(sprite_t *sprite)
{
  st7735_spi_tft_read_rect_short(sprite->x, sprite->width, sprite->y, sprite->height, (uint8_t*)sprite->filled);
  apply_alpha(sprite);
  st7735_spi_tft_write_rect_short(sprite->x, sprite->width, sprite->y, sprite->height,(uint8_t*)sprite->sprite);
}

uint8_t move_sprite(sprite_t *sprite)
{
  uint16_t row[sprite->width], col[sprite->height], tmp[sprite->height];
  uint8_t i;
  int8_t dx, dy;

  if (sprite->direction_stopped) return(0);
  if (!sprite->direction_cur_x && !sprite->direction_cur_y)
  {
  	sprite->direction_cur_x=(sprite->direction_vert?sprite->direction_shift:SPRITE_MAX_SHIFT);
  	sprite->direction_cur_y=(sprite->direction_vert?SPRITE_MAX_SHIFT:sprite->direction_shift);
  }
  dx=(sprite->direction_vert?(sprite->direction_cur_x?(
  		(SPRITE_MAX_SHIFT*SPRITE_CALC_SCALER/sprite->direction_shift>=sprite->direction_cur_y*SPRITE_CALC_SCALER/sprite->direction_cur_x)?1:0):0):1);
  dy=(sprite->direction_vert?1:(sprite->direction_cur_y?(
  		(SPRITE_MAX_SHIFT*SPRITE_CALC_SCALER/sprite->direction_shift>=sprite->direction_cur_x*SPRITE_CALC_SCALER/sprite->direction_cur_y)?1:0):0));
  sprite->direction_cur_x-=dx;
  sprite->direction_cur_y-=dy;

  // Border collision detection
  if (dx)
  {
		if ( ( sprite->direction_right && sprite->x==0 ) ||
		     ( !sprite->direction_right && sprite->x+sprite->width>=ST7735_SCREEN_WIDTH ) )
		{
			sprite->direction_right=!sprite->direction_right;
		}
	  if (sprite->direction_right)	dx= -dx;
  }
  if (dy)
  {
		if ( ( !sprite->direction_up && sprite->y==0 ) ||
				 ( sprite->direction_up && sprite->y+sprite->height>=ST7735_SCREEN_HEIGHT) )
	  {
	   	sprite->direction_up=!sprite->direction_up;
	  }
	  if (!sprite->direction_up) dy= -dy;
  }

  // Read and restore background
  if (dx)
  {
  	if (sprite->height>1)
  	{
      st7735_spi_tft_read_rect_short(sprite->x+(dx>0?sprite->width:-1), 1,
      		sprite->y+(dy>0?1:0), sprite->height-(dy?1:0), (uint8_t*)col);
      for (i=0;i<sprite->height;i++)
      {
      	tmp[i]=*(sprite->filled+(dx>0?0:sprite->width-1)+sprite->width*i);
      }
      st7735_spi_tft_write_rect_short(sprite->x+(dx>0?0:sprite->width-1), 1,
      		sprite->y+(dy>0?1:0), sprite->height-(dy?1:0), (uint8_t*)(tmp+(dy>0?1:0)));
  	} else {
  		if (!dy)
  		{
        st7735_spi_tft_read_rect_short(sprite->x+(dx>0?sprite->width:-1), 1,
        		sprite->y, 1, (uint8_t*)col);
        st7735_spi_tft_write_rect_short(sprite->x+(dx>0?0:sprite->width-1), 1,
        		sprite->y, 1, (uint8_t*)(sprite->filled+(dx>0?0:sprite->width-1)));
  		}
  	}

  }

  if (dy)
  {
  	st7735_spi_tft_read_rect_short(sprite->x+dx, sprite->width,
    		sprite->y+(dy>0?sprite->height:-1), 1, (uint8_t*)row);
    st7735_spi_tft_write_rect_short(sprite->x, sprite->width,
    		sprite->y+(dy>0?0:sprite->height-1), 1,
    		(uint8_t*)(sprite->filled+(dy>0?0:sprite->width*(sprite->height-1))));
  }

  // Justify stored background
  memmove(sprite->filled+(dx>0?(dy<0?sprite->width:0):(dx?(dy<0?sprite->width:0)+1:(dy<0?sprite->width:0))),
  		sprite->filled+(dx>0?(dy>0?sprite->width:0)+1:(dx?(dy>0?sprite->width:0):(dy>0?sprite->width:0))),
  		sprite->width*(sprite->height-(dy?1:0))*2-(dx?2:0));
  if (dy)
  {
  	memcpy(sprite->filled+(dy>0?sprite->width*(sprite->height-1):0), row, sprite->width*2);
  }

  if (dx)
  {
    if ( sprite->height>1 )
    {
      for (i=0;i<sprite->height-(dy?1:0);i++)
      {
        *(sprite->filled+(dx<0?0:sprite->width-1)+(dy<0?sprite->width:0)+sprite->width*i)=col[i];
      }
    } else {
  		if (!dy)
  		{
  			*(sprite->filled+(dx<0?0:sprite->width-1))=col[0];
  		}
    }
  }

  sprite->x+=dx;
  sprite->y+=dy;
  apply_alpha(sprite);
  st7735_spi_tft_write_rect_short(sprite->x, sprite->width, sprite->y, sprite->height, (uint8_t*)sprite->sprite);
  return(0);
}

#undef _ST7735_SPRITE_C_
