/*
 * AVRkanoid.c
 *
 *  Created on: 15 oct. 2016 ã.
 *      Author: ptr
 */

#define _AVRKANOID_C_
#include "AVRkanoid.h"

static void make_border(uint8_t width, uint8_t height, uint16_t body_color, uint16_t border_color, uint16_t *image)
{
	uint8_t i, j;

	for (i=0;i<height;i++)
	{
		for (j=0;j<width;j++)
		{
			if (i==0 || i==height-1)
			{
				if (j==0 || j==width-1)
				{
					image[j+i*width]=(AVRKANOID_BACK_COLOR_START>>8)|(AVRKANOID_BACK_COLOR_START<<8);
				} else {
					image[j+i*width]=(border_color>>8)|(border_color<<8);
				}
			} else {
				if (j==0 || j==width-1)
				{
					image[j+i*width]=(border_color>>8)|(border_color<<8);
				} else {
					image[j+i*width]=(body_color>>8)|(body_color<<8);
				}
			}
		}
	}
  return;
}

uint8_t AVRkanoid_setup(void)
{
	uint8_t rc;

	if ( !(avrkanoid_data=malloc(sizeof(avrkanoid_data_t))) ) return(AVRKANOID_ERR_NOMEMORY);

	if ( !(avrkanoid_data->ball_sprite=malloc(sizeof(sprite_t))) ) return(AVRKANOID_ERR_NOMEMORY);
	if ( !(avrkanoid_data->ball_sprite->sprite=malloc(AVRKANOID_BALL_SIZE*AVRKANOID_BALL_SIZE*2)) ) return(AVRKANOID_ERR_NOMEMORY);
	make_border(AVRKANOID_BALL_SIZE, AVRKANOID_BALL_SIZE, AVRKANOID_BALL_COLOR, AVRKANOID_BALL_BORDER, avrkanoid_data->ball_sprite->sprite);
	avrkanoid_data->ball_sprite->alfa_channel_bitmask=(uint8_t *)AVRKANOID_BALL_ALPHA;
	if ( !(avrkanoid_data->ball_sprite->filled=malloc(AVRKANOID_BALL_SIZE*AVRKANOID_BALL_SIZE*2)) ) return(AVRKANOID_ERR_NOMEMORY);
	avrkanoid_data->ball_sprite->width=AVRKANOID_BALL_SIZE;
	avrkanoid_data->ball_sprite->height=AVRKANOID_BALL_SIZE;

	if ( !(avrkanoid_data->racket_sprite=malloc(sizeof(sprite_t))) ) return(AVRKANOID_ERR_NOMEMORY);
	if ( !(avrkanoid_data->racket_sprite->sprite=malloc(AVRKANOID_RACKET_HEIGHT*AVRKANOID_RACKET_WIDTH*2)) ) return(AVRKANOID_ERR_NOMEMORY);
	make_border(AVRKANOID_RACKET_WIDTH, AVRKANOID_RACKET_HEIGHT, AVRKANOID_RACKET_COLOR, AVRKANOID_RACKET_BORDER, avrkanoid_data->racket_sprite->sprite);
	avrkanoid_data->racket_sprite->alfa_channel_bitmask=(uint8_t *)AVRKANOID_RACKET_ALPHA;
	if ( !(avrkanoid_data->racket_sprite->filled=malloc(AVRKANOID_RACKET_HEIGHT*AVRKANOID_RACKET_WIDTH*2)) ) return(AVRKANOID_ERR_NOMEMORY);
	avrkanoid_data->racket_sprite->width=AVRKANOID_RACKET_WIDTH;
	avrkanoid_data->racket_sprite->height=AVRKANOID_RACKET_HEIGHT;

	if ( !(avrkanoid_data->bricks_bitmask=
			malloc((ST7735_SCREEN_WIDTH/AVRKANOID_BRICK_WIDTH)*AVRKANOID_BRICK_LAYERS/8)) )
		return(AVRKANOID_ERR_NOMEMORY);

	pinMode(AVRKANOID_RIGHT_PIN, INPUT_PULLUP);
	pinMode(AVRKANOID_LEFT_PIN, INPUT_PULLUP);

	st7735_spi_tft_init();
	rc=AVRkanoid_new_game();
  return(rc);
}

uint8_t AVRkanoid_new_game(void)
{
	uint8_t i, y;
	uint16_t c;
	uint16_t *brick;

	avrkanoid_data->ball_sprite->x=ST7735_SCREEN_WIDTH/2-AVRKANOID_BALL_SIZE/2-1;
	avrkanoid_data->ball_sprite->y=AVRKANOID_RACKET_FROM_Y+AVRKANOID_RACKET_HEIGHT;
	avrkanoid_data->ball_sprite->direction_stopped=1;
	avrkanoid_data->ball_sprite->direction_cur_x=0;
	avrkanoid_data->ball_sprite->direction_cur_y=0;
	avrkanoid_data->ball_sprite->direction_right=0;
	avrkanoid_data->ball_sprite->direction_up=1;
	avrkanoid_data->ball_sprite->direction_shift=0;
	avrkanoid_data->ball_sprite->direction_vert=1;

	avrkanoid_data->racket_sprite->x=ST7735_SCREEN_WIDTH/2-AVRKANOID_RACKET_WIDTH/2-1;
	avrkanoid_data->racket_sprite->y=AVRKANOID_RACKET_FROM_Y;
	avrkanoid_data->racket_sprite->direction_stopped=1;
	avrkanoid_data->racket_sprite->direction_cur_x=0;
	avrkanoid_data->racket_sprite->direction_cur_y=0;
	avrkanoid_data->racket_sprite->direction_right=0;
	avrkanoid_data->racket_sprite->direction_up=0;
	avrkanoid_data->racket_sprite->direction_shift=0;
	avrkanoid_data->racket_sprite->direction_vert=0;

  memcpy(avrkanoid_data->bricks_bitmask,AVRKANOID_BRICKS_MASK0,((ST7735_SCREEN_WIDTH/AVRKANOID_BRICK_WIDTH)*AVRKANOID_BRICK_LAYERS)/8);

  c=AVRKANOID_BACK_COLOR_START;
  st7735_spi_tft_fill_rectangle(0, ST7735_SCREEN_WIDTH, ST7735_SCREEN_HEIGHT-AVRKANOID_BRICK_HEIGHT*AVRKANOID_BRICK_LAYERS-AVRKANOID_BRICK_FROM_Y*2,
  		AVRKANOID_BRICK_FROM_Y*2+AVRKANOID_BRICK_HEIGHT*AVRKANOID_BRICK_LAYERS, c);
  c+=(ST7735_SCREEN_HEIGHT-AVRKANOID_BRICK_FROM_Y*2-AVRKANOID_BRICK_HEIGHT*AVRKANOID_BRICK_LAYERS)/AVRKANOID_BACK_COLOR_STEP;
  for ( i=0; i<ST7735_SCREEN_HEIGHT-AVRKANOID_BRICK_FROM_Y*2-AVRKANOID_BRICK_HEIGHT*AVRKANOID_BRICK_LAYERS; i+=AVRKANOID_BACK_COLOR_STEP)
  {
  	st7735_spi_tft_fill_rectangle(0, ST7735_SCREEN_WIDTH, i, AVRKANOID_BACK_COLOR_STEP, c--);
  }
  if ( !(brick=malloc(AVRKANOID_BRICK_WIDTH*AVRKANOID_BRICK_HEIGHT*2)) ) return(AVRKANOID_ERR_NOMEMORY);
	make_border(AVRKANOID_BRICK_WIDTH, AVRKANOID_BRICK_HEIGHT, AVRKANOID_BRICK_COLOR, AVRKANOID_BRICK_BORDER, brick);
	for (i=0;i<(ST7735_SCREEN_WIDTH/AVRKANOID_BRICK_WIDTH)*AVRKANOID_BRICK_LAYERS;i++)
	{
		if ( avrkanoid_data->bricks_bitmask[i/8]&(1<<(i%8)) )
		{
			st7735_spi_tft_write_rect_short((i%(ST7735_SCREEN_WIDTH/AVRKANOID_BRICK_WIDTH))*AVRKANOID_BRICK_WIDTH, AVRKANOID_BRICK_WIDTH,
					ST7735_SCREEN_HEIGHT-AVRKANOID_BRICK_FROM_Y-AVRKANOID_BRICK_HEIGHT-(i/(ST7735_SCREEN_WIDTH/AVRKANOID_BRICK_WIDTH))*AVRKANOID_BRICK_HEIGHT,
					AVRKANOID_BRICK_HEIGHT,	(uint8_t*)brick);
		}
	}
	free(brick);

	avrkanoid_data->frame_delay=AVRKANOID_FRAME_DELAY;
	avrkanoid_data->frame_delay_step=AVRKANOID_FRAME_DELAY_STEP;
	avrkanoid_data->racket_delay=0;

	draw_sprite(avrkanoid_data->racket_sprite);
	draw_sprite(avrkanoid_data->ball_sprite);

	return(0);
}

static uint8_t check_one_brick_collision(uint8_t x, uint8_t y)
{
	uint8_t brick_no;

	if ( y>ST7735_SCREEN_HEIGHT-AVRKANOID_BRICK_FROM_Y ||
			 y<ST7735_SCREEN_HEIGHT-AVRKANOID_BRICK_FROM_Y-AVRKANOID_BRICK_LAYERS*AVRKANOID_BRICK_HEIGHT-AVRKANOID_BALL_SIZE)
	{
		return(0);
	}

	brick_no=(ST7735_SCREEN_WIDTH/AVRKANOID_BRICK_WIDTH)*(AVRKANOID_BRICK_LAYERS-1-((y-(ST7735_SCREEN_HEIGHT-AVRKANOID_BRICK_FROM_Y-AVRKANOID_BRICK_LAYERS*AVRKANOID_BRICK_HEIGHT))/
			AVRKANOID_BRICK_HEIGHT))+x/AVRKANOID_BRICK_WIDTH;
	if ( !(avrkanoid_data->bricks_bitmask[brick_no/8]&(1<<(brick_no%8))) )
	{
		return(0);
	}
	avrkanoid_data->bricks_bitmask[brick_no/8]&=(~(1<<(brick_no%8)));
	st7735_spi_tft_fill_rectangle(brick_no%(ST7735_SCREEN_WIDTH/AVRKANOID_BRICK_WIDTH)*AVRKANOID_BRICK_WIDTH, AVRKANOID_BRICK_WIDTH,
			ST7735_SCREEN_HEIGHT-AVRKANOID_BRICK_FROM_Y-AVRKANOID_BRICK_HEIGHT-(brick_no/(ST7735_SCREEN_WIDTH/AVRKANOID_BRICK_WIDTH))*AVRKANOID_BRICK_HEIGHT,
			AVRKANOID_BRICK_HEIGHT,	AVRKANOID_BACK_COLOR_START);
	return(1);
}

static uint8_t check_brick_collision(void)
{
  uint8_t i, rc;

  i=rc=0;
  if (avrkanoid_data->ball_sprite->direction_up)
  {
    i|=check_one_brick_collision(avrkanoid_data->ball_sprite->x,avrkanoid_data->ball_sprite->y+AVRKANOID_BALL_SIZE);
    i|=check_one_brick_collision(avrkanoid_data->ball_sprite->x+AVRKANOID_BALL_SIZE-1,avrkanoid_data->ball_sprite->y+AVRKANOID_BALL_SIZE);
  } else {
    i|=check_one_brick_collision(avrkanoid_data->ball_sprite->x,avrkanoid_data->ball_sprite->y-1);
    i|=check_one_brick_collision(avrkanoid_data->ball_sprite->x+AVRKANOID_BALL_SIZE-1,avrkanoid_data->ball_sprite->y-1);
  }
  if (i)
  {
  	avrkanoid_data->ball_sprite->direction_up=!avrkanoid_data->ball_sprite->direction_up;
  	rc++;	i=0;
  }

  if (avrkanoid_data->ball_sprite->direction_right)
  {
  	if (avrkanoid_data->ball_sprite->x<ST7735_SCREEN_WIDTH-AVRKANOID_BALL_SIZE)
  	{
  		i|=check_one_brick_collision(avrkanoid_data->ball_sprite->x-1,avrkanoid_data->ball_sprite->y);
  		i|=check_one_brick_collision(avrkanoid_data->ball_sprite->x-1,avrkanoid_data->ball_sprite->y+AVRKANOID_BALL_SIZE-1);
  	}
  } else {
  	if (avrkanoid_data->ball_sprite->x)
  	{
  		i|=check_one_brick_collision(avrkanoid_data->ball_sprite->x+AVRKANOID_BALL_SIZE,avrkanoid_data->ball_sprite->y);
  		i|=check_one_brick_collision(avrkanoid_data->ball_sprite->x+AVRKANOID_BALL_SIZE,avrkanoid_data->ball_sprite->y+AVRKANOID_BALL_SIZE-1);
  	}
  }
  if (i)
  {
  	avrkanoid_data->ball_sprite->direction_right=!avrkanoid_data->ball_sprite->direction_right;
  	rc++;
  }

  if ( !rc && avrkanoid_data->ball_sprite->direction_shift )
  {
  	if (avrkanoid_data->ball_sprite->direction_right)
  	{
    	if (avrkanoid_data->ball_sprite->x)
    	{
    		rc|=check_one_brick_collision(avrkanoid_data->ball_sprite->x-1,
    				(avrkanoid_data->ball_sprite->direction_up?avrkanoid_data->ball_sprite->y+AVRKANOID_BALL_SIZE:avrkanoid_data->ball_sprite->y-1));
    	}
  	} else {
    	if (avrkanoid_data->ball_sprite->x<ST7735_SCREEN_WIDTH-AVRKANOID_BALL_SIZE)
    	{
    		rc|=check_one_brick_collision(avrkanoid_data->ball_sprite->x+AVRKANOID_BALL_SIZE,
    				(avrkanoid_data->ball_sprite->direction_up?avrkanoid_data->ball_sprite->y+AVRKANOID_BALL_SIZE:avrkanoid_data->ball_sprite->y-1));
    	}
  	}
    if (rc)
    {
    	avrkanoid_data->ball_sprite->direction_right=!avrkanoid_data->ball_sprite->direction_right;
    	avrkanoid_data->ball_sprite->direction_up=!avrkanoid_data->ball_sprite->direction_up;
    }
  }

  if (!rc)
  {
  	return(0);
  }

 	for ( i=0; i<((ST7735_SCREEN_WIDTH/AVRKANOID_BRICK_WIDTH)*AVRKANOID_BRICK_LAYERS)/8; i++)
	{
		if ( avrkanoid_data->bricks_bitmask[i] ) return(0);
	}
	return(1);
}

static uint8_t check_racket_collision(void)
{
	int16_t mid_ball, mid_racket;
	int8_t shift;

	avrkanoid_data->ball_sprite->direction_up=1;
	if ( (avrkanoid_data->ball_sprite->x+AVRKANOID_BALL_SIZE-1==avrkanoid_data->racket_sprite->x) &&
			 avrkanoid_data->ball_sprite->direction_shift && !avrkanoid_data->ball_sprite->direction_right )
	{
		avrkanoid_data->ball_sprite->direction_right=1;
		return(0);
	}

	if ( (avrkanoid_data->ball_sprite->x==avrkanoid_data->racket_sprite->x+AVRKANOID_RACKET_WIDTH-1) &&
			 avrkanoid_data->ball_sprite->direction_shift && avrkanoid_data->ball_sprite->direction_right )
	{
		avrkanoid_data->ball_sprite->direction_right=0;
		return(0);
	}

  if ( (avrkanoid_data->ball_sprite->x+AVRKANOID_BALL_SIZE-1<avrkanoid_data->racket_sprite->x) ||
  		 (avrkanoid_data->ball_sprite->x>avrkanoid_data->racket_sprite->x+AVRKANOID_RACKET_WIDTH-1) )
  {
  	return(1);
  }

  mid_ball=avrkanoid_data->ball_sprite->x+AVRKANOID_BALL_SIZE/2;
  mid_racket=avrkanoid_data->racket_sprite->x+AVRKANOID_RACKET_WIDTH/2;
  if ( abs(mid_ball-mid_racket)>1 )
  {
  	if (avrkanoid_data->ball_sprite->direction_shift)
  	{
    	shift=(int8_t)avrkanoid_data->ball_sprite->direction_shift+abs(mid_ball-mid_racket)*(mid_ball>mid_racket?1:-1);
    	avrkanoid_data->ball_sprite->direction_shift=(shift<0?0:(shift>SPRITE_MAX_SHIFT?SPRITE_MAX_SHIFT:shift));
  	} else {
    	shift=abs(mid_ball-mid_racket);
    	avrkanoid_data->ball_sprite->direction_shift=shift>SPRITE_MAX_SHIFT?SPRITE_MAX_SHIFT:shift;
 			avrkanoid_data->ball_sprite->direction_right=mid_ball>mid_racket?0:1;
  	}
		avrkanoid_data->ball_sprite->direction_cur_x=0;
		avrkanoid_data->ball_sprite->direction_cur_y=0;
  }

	return(0);
}

void AVRkanoid_loop(void)
{
	uint8_t frame_start, frame_len;

	frame_start=(micros()>>AVRKANOID_DELAY_SCALER)&0xFF;
	move_sprite(avrkanoid_data->ball_sprite);
	if ( !avrkanoid_data->ball_sprite->direction_up &&
			avrkanoid_data->ball_sprite->y==AVRKANOID_RACKET_FROM_Y+AVRKANOID_RACKET_HEIGHT )
	{
		if ( check_racket_collision() )
		{
			delay(AVRKANOID_GAME_FINISH_DELAY);
			AVRkanoid_new_game();
			return;
		}
	} else {
		if ( avrkanoid_data->ball_sprite->y>=ST7735_SCREEN_HEIGHT-AVRKANOID_BRICK_FROM_Y-AVRKANOID_BRICK_LAYERS*AVRKANOID_BRICK_HEIGHT-AVRKANOID_BALL_SIZE &&
				 avrkanoid_data->ball_sprite->y<=ST7735_SCREEN_HEIGHT-AVRKANOID_BRICK_FROM_Y)
		{
			if ( check_brick_collision() )
			{
				delay(AVRKANOID_GAME_FINISH_DELAY);
				AVRkanoid_new_game();
				return;
			}
		}
	}

	if ( avrkanoid_data->racket_delay )
	{
		avrkanoid_data->racket_delay--;
	} else {
		if (digitalRead(AVRKANOID_RIGHT_PIN)==LOW)
		{
			avrkanoid_data->racket_sprite->direction_stopped=0;
			avrkanoid_data->racket_sprite->direction_right=1;
			avrkanoid_data->ball_sprite->direction_stopped=0;
			move_sprite(avrkanoid_data->racket_sprite);
			avrkanoid_data->racket_delay=(avrkanoid_data->frame_delay>=AVRKANOID_RACKET_DELAY?0:1);
		} else {
			if (digitalRead(AVRKANOID_LEFT_PIN)==LOW)
			{
				avrkanoid_data->racket_sprite->direction_stopped=0;
				avrkanoid_data->racket_sprite->direction_right=0;
				avrkanoid_data->ball_sprite->direction_stopped=0;
				move_sprite(avrkanoid_data->racket_sprite);
				avrkanoid_data->racket_delay=(avrkanoid_data->frame_delay>=AVRKANOID_RACKET_DELAY?0:1);
			}
		}
	}

	frame_len=(uint8_t)((micros()>>AVRKANOID_DELAY_SCALER)&0xFF)-frame_start;
	if (frame_len<avrkanoid_data->frame_delay)
	{
		delayMicroseconds(((uint16_t)(avrkanoid_data->frame_delay-frame_len))<<AVRKANOID_DELAY_SCALER);
	}
	return;
}


#undef _AVRKANOID_C_
