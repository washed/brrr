/*
 * oled.c
 *
 *  Created on: 07.06.2017
 *      Author: washed
 */

#include "main.h"
#include "stm32f3xx_hal.h"
#include "oled.h"

static volatile uint8_t display_shadow[WIDTH * HEIGHT / 8];

void set_mode_data() {
	HAL_GPIO_WritePin( OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);
}

void set_mode_command() {
	HAL_GPIO_WritePin( OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);
}

void select() {
	HAL_GPIO_WritePin( OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET);
}

void deselect() {
	HAL_GPIO_WritePin( OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);
}

void reset() {
	HAL_GPIO_WritePin( OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET);
}

void not_reset() {
	HAL_GPIO_WritePin( OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET);
}

void write_byte(uint8_t byte) {
	HAL_SPI_Transmit( OLED_SPI_INSTANCE_PTR, &byte, 1, 10);
}

void initialize() {
	select();
	not_reset();
	HAL_Delay(10);
	reset();
	HAL_Delay(10);
	not_reset();

	set_mode_command();

	write_byte(0xAE); //--turn off oled panel
	write_byte(0x02); //---set low column address
	write_byte(0x10); //---set high column address
	write_byte(0x40); //--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	write_byte(0xA1); //--Set SEG/Column Mapping
	write_byte(0xC8); //Set COM/Row Scan Direction
	write_byte(0xA6); //--set normal display
	write_byte(0xA8); //--set multiplex ratio(1 to 64)
	write_byte(0x3F); //--1/64 duty
	write_byte(0xD3); //-set display offset    Shift Mapping RAM Counter (0x00~0x3F)
	write_byte(0x00); //-not offset
	write_byte(0xd5); //--set display clock divide ratio/oscillator frequency
	write_byte(0x80); //--set divide ratio, Set Clock as 100 Frames/Sec
	write_byte(0xAD); //--set charge pump
	write_byte(0x8B); //set Charge Pump enable
	write_byte(0xDA); //--set com pins hardware configuration
	write_byte(0x12);
	write_byte(0x81); //--set contrast control register
	write_byte(0x80);
	write_byte(0xD9); //--set pre-charge period
	write_byte(0x22); //Set Pre-Charge
	write_byte(0xDB); //--set vcomh
	write_byte(0x40); //Set VCOM Deselect Level
	write_byte(0x20); //-Set Page Addressing Mode (0x00/0x01/0x02)
	write_byte(0x02); //
	write_byte(0xA4); // Disable Entire Display On (0xa4/0xa5)
	write_byte(0xA6); // Disable Inverse Display On (0xa6/a7)
	write_byte(0xAF); //--turn on oled panel

	deselect();
}

void my_disp_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p)
{
    int32_t x, y;
    for(y = area->y1; y <= area->y2; y++) {
        for(x = area->x1; x <= area->x2; x++) {
            // set_pixel(x, y, (uint8_t)(*(uint8_t*)color_p));  /* Put a pixel to the display.*/


        	set_pixel(x, y, color_p->full);

            color_p++;
        }
    }
    // TODO: We're doing some double buffering here, might not be needed!
    display();
    lv_disp_flush_ready(disp);         /* Indicate you are ready with the flushing*/
}

void display() {
	select();

	uint8_t *shadow_pointer = (uint8_t*) &display_shadow;
	for (uint8_t page = 0; page < PAGES; page++) {
		set_mode_command();
		write_byte(0xB0 + page);
		write_byte(0x02);
		write_byte(0x10);

		set_mode_data();
		HAL_SPI_Transmit( OLED_SPI_INSTANCE_PTR, shadow_pointer, WIDTH, 10);
		shadow_pointer += WIDTH;
	}
	deselect();
}

void fill(uint8_t value) {
	for (uint32_t i = 0; i < 1024; i++)
		display_shadow[i] = value;
}

void set_pixel(int32_t x, int32_t y, uint8_t color) {
	if (x > WIDTH || y > HEIGHT)
		return;

	if (color ^ INVERT)
		display_shadow[x + (y / 8) * WIDTH] |= 1 << (y % 8);
	else
		display_shadow[x + (y / 8) * WIDTH] &= ~(1 << (y % 8));
}
