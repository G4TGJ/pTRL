/*
 * WM8960.c
 *
 * Created: 14/01/2025
 * Author : Richard Tomlinson G4TGJ
 */ 

#include "config.h"
#include "i2c.h"
#include "WM8960.h"

// Keep track of ADC volumes
static uint8_t leftADCVolume, rightADCVolume;

uint8_t WM8960GetLeftADCVolume( void )
{
    return leftADCVolume;
}

uint8_t WM8960GetRightADCVolume( void )
{
    return rightADCVolume;
}

void WM8960SetLeftADCVolume( uint8_t vol )
{
    leftADCVolume = vol;
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_LEFT_ADC_VOLUME,  (uint16_t)0x000 | leftADCVolume);
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_LEFT_ADC_VOLUME,  (uint16_t)0x100 | leftADCVolume);
}

void WM8960SetRightADCVolume( uint8_t vol )
{
    rightADCVolume = vol;
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_RIGHT_ADC_VOLUME, (uint16_t)0x000 | rightADCVolume);
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_RIGHT_ADC_VOLUME, (uint16_t)0x100 | rightADCVolume);
}

void WM8960Init( void )
{
    // Initialise I2C first
    i2cInit();

    // Set up clocks for 48kHz sample rate
    // MCLK on Sparkfun board is 24MHz
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_PLL_N, 0x78);
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_CLOCKING_1, 0x05);
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_CLOCKING_2, 0x04);
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_PLL_K_1, 0x31);
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_PLL_K_2, 0x26);
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_PLL_K_3, 0xE8);

    // Enable VREF, set VMID, power up ADCs
    // Enable left and right boost mixers
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_PWR_MGMT_1, 0xFC);

    // Enable LINPUT3 and RINPUT3 with 6dB gain
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_INPUT_BOOST_MIXER_1, 0x70);
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_INPUT_BOOST_MIXER_2, 0x70);

    // Set left and right ADC volume
    WM8960SetLeftADCVolume( DEFAULT_LEFT_ADC_VOLUME );
    WM8960SetRightADCVolume( DEFAULT_RIGHT_ADC_VOLUME );

    // Enable left and right output mixers
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_PWR_MGMT_3, 0x0C);

    // Enable headphone output including OUT3 for virtual ground
    // Enable PLL
    // Power up DACs
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_PWR_MGMT_2, 0x1E3);

    // Set 16 bit I2S and enable master mode
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_AUDIO_INTERFACE_1, 0x42);

    // Unmute the DAC
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_ADC_DAC_CTRL_1, 0x00);

    // Enable loopback (ADC output connected to DAC input)
    //i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_AUDIO_INTERFACE_2, 0x01);

    // Set ADCLRC as GPIO
    //i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_AUDIO_INTERFACE_2, 0x40);

    // Enable left and right DAC outputs
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_LEFT_OUT_MIX_1, 0x100);
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_RIGHT_OUT_MIX_2, 0x100);
}

void WM8960SetHeadphoneVolume( void )
{
    // Set headphone volume
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_LOUT1_VOLUME, 0x1FF);
    i2cWriteRegister9Bit(WM8960_ADDR, WM8960_REG_ROUT1_VOLUME, 0x1FF);
}