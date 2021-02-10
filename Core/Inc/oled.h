/*
 * oled.h
 *
 *  Created on: Feb 7, 2021
 *      Author: Marc
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

#include "lvgl/lvgl.h"

#define INVERT 1
#define WIDTH 128
#define HEIGHT 64
#define PAGES HEIGHT / 8
#define OLED_SPI_INSTANCE_PTR &hspi2

void initialize();
void start_lvgl_tick();
void set_pixel(int32_t x, int32_t y, uint8_t color);
void my_disp_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p);
void display();


#endif /* INC_OLED_H_ */
