/*
 * lcd_port.c
 *
 * Low level functions for writing to LCD display over an IO port
 *
 * Created: 23/07/2023
 * Author : Richard Tomlinson G4TGJ
*/ 
#include "hardware/gpio.h"

#include "config.h"
#include "lcd.h"

static void gpioSetOutput( uint gpio )
{
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_OUT);
}

// Initialise the LCD interface
void lcdIFInit()
{
    // Set up LCD pins as outputs
    gpioSetOutput(LCD_RS_GPIO);
    gpioSetOutput(LCD_ENABLE_GPIO);
    gpioSetOutput(LCD_DATA_GPIO_0);
    gpioSetOutput(LCD_DATA_GPIO_1);
    gpioSetOutput(LCD_DATA_GPIO_2);
    gpioSetOutput(LCD_DATA_GPIO_3);

    // RW port is often not used
#ifdef LCD_RW_GPIO
    gpioSetOutput(LCD_RW_GPIO);
#endif
}

// Write to the LCD's data bits
void lcdWriteData( uint8_t value )
{
    gpio_put(LCD_DATA_GPIO_0, (value >> 0) & 0x01);
    gpio_put(LCD_DATA_GPIO_1, (value >> 1) & 0x01);
    gpio_put(LCD_DATA_GPIO_2, (value >> 2) & 0x01);
    gpio_put(LCD_DATA_GPIO_3, (value >> 3) & 0x01);
}

// Set or clear the RS bit
void lcdRS( bool bOn )
{
    gpio_put(LCD_RS_GPIO, bOn);
}

// Set or clear the EN bit
void lcdEN( bool bOn )
{
    gpio_put(LCD_ENABLE_GPIO, bOn);
}

// Set or clear the RW bit
// This bit is not always used
void lcdRW( bool bOn )
{
#ifdef LCD_RW_GPIO
    gpio_put(LCD_RW_GPIO, bOn);
#endif
}

// Set or clear the backlight bit
void lcdBacklight( bool bOn )
{
}
