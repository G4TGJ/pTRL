/*
 * i2c.c
 *
 * I2C driver
 *
 * Created: 13/07/2023
 * Author : Richard Tomlinson G4TGJ
 */ 
#include <string.h>
#include <stdio.h>

#include "config.h"
#include "io.h"
#include "i2c.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

// Inline functions to extract the most and least significant bytes from
// a 16 bit value.
inline uint8_t msb( uint16_t val)
{
    return (val&0xFF00)>>8;
}

inline uint8_t lsb( uint16_t val)
{
    return (val&0x00FF);
}

// Wait for an I2C address to acknowledge
// Used to wait for a device to be ready
// e.g EEPROM write
void i2cWait( uint8_t i2cAddr )
{
    int ret = -1;
    uint8_t dummy;
    int count = 0;
    while( ret < 0 && count < MAX_I2C_WAIT_COUNT )
    {
        ret = i2c_read_blocking(i2c_default, i2cAddr, &dummy, 1, false);
        count++;
    }
}

// Write an 8 bit byte to I2C
uint8_t i2cWriteByte(uint8_t i2cAddr, uint8_t data)
{
    i2c_write_blocking(i2c_default, i2cAddr, &data, 1, false);

    return 0;
}

// Functions to read and write an 8 bit register at an 8 bit register address
uint8_t i2cWriteRegister(uint8_t i2cAddr, uint8_t regAddr, uint8_t data)
{
    uint8_t buf[2] = { regAddr, data };
    printf("Writing %d:0x%02x\n", regAddr, data);
    i2c_write_blocking(i2c_default, i2cAddr, buf, 2, false);

    return 0;
}

// Write to a 9 bit register
// First byte written contains the reg address shifted left one bit
// plus bit 8 of data.
// Second byte written contains the remaining 8 bits of the data
uint8_t i2cWriteRegister9Bit(uint8_t i2cAddr, uint8_t regAddr, uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (regAddr << 1) | (data>>8);
    buf[1] = data & 0xFF;
    i2c_write_blocking(i2c_default, i2cAddr, buf, 2, false);

    return 0;
}

uint8_t i2cReadRegister(uint8_t i2cAddr, uint8_t regAddr, uint8_t *data)
{
    i2c_write_blocking(i2c_default, i2cAddr, &regAddr, 1, true);
    i2c_read_blocking(i2c_default, i2cAddr, data, 1, false);
    printf("Read %d:0x%02x\n", regAddr, *data);

    return 0;
}

// Functions to read and write an 8 bit register at a 16 bit register address
uint8_t i2cWriteDataA16(uint8_t i2cAddr, uint16_t dataAddr, uint8_t *data, int num)
{
    // I2C only works in 8 bit so we have to split the 16 bit address
    // in to two bytes. I2C devices appear to be big endian so need
    // the msb first. This is then followed by the data to write.
    static uint8_t buf[MAX_I2C_DATA_WRITE + 2];

    if( num <= MAX_I2C_DATA_WRITE )
    {
        buf[0] = msb(dataAddr);
        buf[1] = lsb(dataAddr);

        memcpy( &buf[2], data, num );

        int ret2 = i2c_write_blocking(i2c_default, i2cAddr, buf, num + 2, false);
    }
    return 0;
}

uint8_t i2cWriteRegisterA16(uint8_t i2cAddr, uint16_t regAddr, uint8_t data)
{
    return i2cWriteDataA16(i2cAddr, regAddr, &data, 1);
}

uint8_t i2cReadDataA16(uint8_t i2cAddr, uint16_t dataAddr, uint8_t *data, int num)
{
    // I2C only works in 8 bit so we have to split the 16 bit address
    // in to two bytes. I2C devices appear to be big endian so need
    // the msb first.
    uint8_t buf[2];
    buf[0] = msb(dataAddr);
    buf[1] = lsb(dataAddr);

    i2c_write_blocking(i2c_default, i2cAddr, buf, 2, true);
    i2c_read_blocking(i2c_default, i2cAddr, data, num, false);

    return 0;
}

uint8_t i2cReadRegisterA16(uint8_t i2cAddr, uint16_t regAddr, uint8_t *data)
{
    return i2cReadDataA16( i2cAddr, regAddr, data, 1 );
}

// Init TWI (I2C)
//
void i2cInit()
{
    // Only allow I2C to be initialised once
    static bool bInit = false;

    if( !bInit )
    {
        bInit = true;

        i2c_init(i2c_default, I2C_CLOCK_RATE);

        gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
        gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

        // Make the I2C pins available to picotool
        bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
    }
}
