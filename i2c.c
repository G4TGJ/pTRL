/*
 * i2c.c
 *
 * I2C driver
 *
 * Created: 13/07/2023
 * Author : Richard Tomlinson G4TGJ
 */ 
 
#include "config.h"
#include "i2c.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

uint8_t i2cWriteRegister(uint8_t addr, uint8_t reg, uint8_t data)
{
    i2c_write_blocking(i2c_default, addr, &reg, 1, true);
    i2c_write_blocking(i2c_default, addr, &data, 1, false);

    return 0;
}

uint8_t i2cReadRegister(uint8_t addr, uint8_t reg, uint8_t *data)
{
    i2c_write_blocking(i2c_default, addr, &reg, 1, true);
    i2c_read_blocking(i2c_default, addr, data, 1, false);

    return 0;
}

// Init TWI (I2C)
//
void i2cInit()
{
    i2c_init(i2c_default, I2C_CLOCK_RATE);

    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
}
