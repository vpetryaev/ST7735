# ST7735

I tried to read frame buffer memory from ST7735 TFT controller. As you can see - succesfuly.
This project contains:

1. ST7735_spi library for fast bidirectional communication with ST7735. Time critical parts writen on AVR assembler.

2. ST7735_sprite library based on previous ST7735_spi. Driving sprites. Support alpha channel for sprites. Backround, filled by sprite, read from framebuffer and restored, while sprite is moving.

3. AVRcanoid - sample game, based on old Arcanoid game. Proposed as sample and how to using ST7735 libraries.

If you want to build this project using Arduino IDE, rename ST7735.c to ST7735.ino
