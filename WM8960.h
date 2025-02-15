/******************************************************************************
  Modified by Richard Tomlinson G4TGJ
 
  SparkFun WM8960 Arduino Library

  This library provides a set of functions to control (via I2C) the Wolfson 
  Microelectronics WM8960 Stereo CODEC with 1W Stereo Class D Speaker Drivers 
  and Headphone Drivers.

  Pete Lewis @ SparkFun Electronics
  October 14th, 2022
  https://github.com/sparkfun/SparkFun_WM8960_Arduino_Library
  
  This code was created using some code by Mike Grusin at SparkFun Electronics
  Included with the LilyPad MP3 example code found here:
  Revision history: version 1.0 2012/07/24 MDG Initial release
  https://github.com/sparkfun/LilyPad_MP3_Player

  Do you like this library? Help support SparkFun. Buy a board!

    SparkFun Audio Codec Breakout - WM8960 (QWIIC)
    https://www.sparkfun.com/products/21250
	
	All functions return 1 if the read/write was successful, and 0
	if there was a communications failure. You can ignore the return value
	if you just don't care anymore.

	For information on the data sent to and received from the CODEC,
	refer to the WM8960 datasheet at:
	https://github.com/sparkfun/SparkFun_Audio_Codec_Breakout_WM8960/blob/main/Documents/WM8960_datasheet_v4.2.pdf

  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
  Please review the LICENSE.md file included with this example. If you have any questions 
  or concerns with licensing, please contact techsupport@sparkfun.com.
  Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __SPARKFUN_WM8960_H__
#define __SPARKFUN_WM8960_H__

// I2C address
#define WM8960_ADDR 0x1A 

// WM8960 register addresses
#define WM8960_REG_LEFT_INPUT_VOLUME 0x00
#define WM8960_REG_RIGHT_INPUT_VOLUME 0x01
#define WM8960_REG_LOUT1_VOLUME 0x02
#define WM8960_REG_ROUT1_VOLUME 0x03
#define WM8960_REG_CLOCKING_1 0x04
#define WM8960_REG_ADC_DAC_CTRL_1 0x05
#define WM8960_REG_ADC_DAC_CTRL_2 0x06
#define WM8960_REG_AUDIO_INTERFACE_1 0x07
#define WM8960_REG_CLOCKING_2 0x08
#define WM8960_REG_AUDIO_INTERFACE_2 0x09
#define WM8960_REG_LEFT_DAC_VOLUME 0x0A
#define WM8960_REG_RIGHT_DAC_VOLUME 0x0B
#define WM8960_REG_RESET 0x0F
#define WM8960_REG_3D_CONTROL 0x10
#define WM8960_REG_ALC1 0x11
#define WM8960_REG_ALC2 0x12
#define WM8960_REG_ALC3 0x13
#define WM8960_REG_NOISE_GATE 0x14
#define WM8960_REG_LEFT_ADC_VOLUME 0x15
#define WM8960_REG_RIGHT_ADC_VOLUME 0x16
#define WM8960_REG_ADDITIONAL_CONTROL_1 0x17
#define WM8960_REG_ADDITIONAL_CONTROL_2 0x18
#define WM8960_REG_PWR_MGMT_1 0x19
#define WM8960_REG_PWR_MGMT_2 0x1A
#define WM8960_REG_ADDITIONAL_CONTROL_3 0x1B
#define WM8960_REG_ANTI_POP_1 0x1C
#define WM8960_REG_ANTI_POP_2 0x1D
#define WM8960_REG_ADCL_SIGNAL_PATH 0x20
#define WM8960_REG_ADCR_SIGNAL_PATH 0x21
#define WM8960_REG_LEFT_OUT_MIX_1 0x22
#define WM8960_REG_RIGHT_OUT_MIX_2 0x25
#define WM8960_REG_MONO_OUT_MIX_1 0x26
#define WM8960_REG_MONO_OUT_MIX_2 0x27
#define WM8960_REG_LOUT2_VOLUME 0x28
#define WM8960_REG_ROUT2_VOLUME 0x29
#define WM8960_REG_MONO_OUT_VOLUME 0x2A
#define WM8960_REG_INPUT_BOOST_MIXER_1 0x2B
#define WM8960_REG_INPUT_BOOST_MIXER_2 0x2C
#define WM8960_REG_BYPASS_1 0x2D
#define WM8960_REG_BYPASS_2 0x2E
#define WM8960_REG_PWR_MGMT_3 0x2F
#define WM8960_REG_ADDITIONAL_CONTROL_4 0x30
#define WM8960_REG_CLASS_D_CONTROL_1 0x31
#define WM8960_REG_CLASS_D_CONTROL_3 0x33
#define WM8960_REG_PLL_N 0x34
#define WM8960_REG_PLL_K_1 0x35
#define WM8960_REG_PLL_K_2 0x36
#define WM8960_REG_PLL_K_3 0x37

// PGA input selections
#define WM8960_PGAL_LINPUT2 0
#define WM8960_PGAL_LINPUT3 1
#define WM8960_PGAL_VMID 2
#define WM8960_PGAR_RINPUT2 0
#define WM8960_PGAR_RINPUT3 1
#define WM8960_PGAR_VMID 2

// Mic (aka PGA) BOOST gain options
#define WM8960_MIC_BOOST_GAIN_0DB 0
#define WM8960_MIC_BOOST_GAIN_13DB 1
#define WM8960_MIC_BOOST_GAIN_20DB 2
#define WM8960_MIC_BOOST_GAIN_29DB 3

// Boost Mixer gain options
// These are used to control the gain (aka volume) at the following settings:
// LIN2BOOST
// LIN3BOOST
// RIN2BOOST
// RIN3BOOST
#define WM8960_BOOST_MIXER_GAIN_MUTE 0
#define WM8960_BOOST_MIXER_GAIN_NEG_12DB 1
#define WM8960_BOOST_MIXER_GAIN_NEG_9DB 2
#define WM8960_BOOST_MIXER_GAIN_NEG_6DB 3
#define WM8960_BOOST_MIXER_GAIN_NEG_3DB 4
#define WM8960_BOOST_MIXER_GAIN_0DB 5
#define WM8960_BOOST_MIXER_GAIN_3DB 6
#define WM8960_BOOST_MIXER_GAIN_6DB 7

// Output Mixer gain options
// These are used to control the gain (aka volume) at the following settings:
// LI2LOVOL
// LB2LOVOL
// RI2LOVOL
// RB2LOVOL
// These are useful as analog bypass signal path options.
#define WM8960_OUTPUT_MIXER_GAIN_0DB 0
#define WM8960_OUTPUT_MIXER_GAIN_NEG_3DB 1
#define WM8960_OUTPUT_MIXER_GAIN_NEG_6DB 2
#define WM8960_OUTPUT_MIXER_GAIN_NEG_9DB 3
#define WM8960_OUTPUT_MIXER_GAIN_NEG_12DB 4
#define WM8960_OUTPUT_MIXER_GAIN_NEG_15DB 5
#define WM8960_OUTPUT_MIXER_GAIN_NEG_18DB 6
#define WM8960_OUTPUT_MIXER_GAIN_NEG_21DB 7

// Mic Bias voltage options
#define WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD 0
#define WM8960_MIC_BIAS_VOLTAGE_0_65_AVDD 1

// SYSCLK divide
#define WM8960_SYSCLK_DIV_BY_1 0
#define WM8960_SYSCLK_DIV_BY_2 2
#define WM8960_CLKSEL_MCLK 0
#define WM8960_CLKSEL_PLL 1
#define WM8960_PLL_MODE_INTEGER 0
#define WM8960_PLL_MODE_FRACTIONAL 1
#define WM8960_PLLPRESCALE_DIV_1 0
#define WM8960_PLLPRESCALE_DIV_2 1

// Class d clock divide
#define WM8960_DCLKDIV_16 7

// Word length settings (aka bits per sample)
// Audio Data Word Length
#define WM8960_WL_16BIT 0
#define WM8960_WL_20BIT 1
#define WM8960_WL_24BIT 2
#define WM8960_WL_32BIT 3

// Additional Digital Audio Interface controls
// LRP (aka left-right-polarity)
// Right, left and I2S modes – LRCLK polarity 
// 0 = normal LRCLK polarity 
// 1 = inverted LRCLK polarity 
#define WM8960_LR_POLARITY_NORMAL 0
#define WM8960_LR_POLARITY_INVERT 1

// ALRSWAP (aka ADC left/right swap)
// Left/Right ADC channel swap
// 1 = Swap left and right ADC data in audio interface
// 0 = Output left and right data as normal 
#define WM8960_ALRSWAP_NORMAL 0
#define WM8960_ALRSWAP_SWAP 1

// Gain mins, maxes, offsets and step-sizes for all the amps within the codec.
#define WM8960_PGA_GAIN_MIN -17.25
#define WM8960_PGA_GAIN_MAX 30.00
#define WM8960_PGA_GAIN_OFFSET 17.25
#define WM8960_PGA_GAIN_STEPSIZE 0.75
#define WM8960_HP_GAIN_MIN -73.00
#define WM8960_HP_GAIN_MAX 6.00
#define WM8960_HP_GAIN_OFFSET 121.00
#define WM8960_HP_GAIN_STEPSIZE 1.00
#define WM8960_SPEAKER_GAIN_MIN -73.00
#define WM8960_SPEAKER_GAIN_MAX 6.00
#define WM8960_SPEAKER_GAIN_OFFSET 121.00
#define WM8960_SPEAKER_GAIN_STEPSIZE 1.00
#define WM8960_ADC_GAIN_MIN -97.00
#define WM8960_ADC_GAIN_MAX 30.00
#define WM8960_ADC_GAIN_OFFSET 97.50
#define WM8960_ADC_GAIN_STEPSIZE 0.50
#define WM8960_DAC_GAIN_MIN -97.00
#define WM8960_DAC_GAIN_MAX 30.00
#define WM8960_DAC_GAIN_OFFSET 97.50
#define WM8960_DAC_GAIN_STEPSIZE 0.50

// Automatic Level Control Modes
#define WM8960_ALC_MODE_OFF 0
#define WM8960_ALC_MODE_RIGHT_ONLY 1
#define WM8960_ALC_MODE_LEFT_ONLY 2
#define WM8960_ALC_MODE_STEREO 3

// Automatic Level Control Target Level dB
#define WM8960_ALC_TARGET_LEVEL_NEG_22_5DB 0
#define WM8960_ALC_TARGET_LEVEL_NEG_21DB 1
#define WM8960_ALC_TARGET_LEVEL_NEG_19_5DB 2
#define WM8960_ALC_TARGET_LEVEL_NEG_18DB 3
#define WM8960_ALC_TARGET_LEVEL_NEG_16_5DB 4
#define WM8960_ALC_TARGET_LEVEL_NEG_15DB 5
#define WM8960_ALC_TARGET_LEVEL_NEG_13_5DB 6
#define WM8960_ALC_TARGET_LEVEL_NEG_12DB 7
#define WM8960_ALC_TARGET_LEVEL_NEG_10_5DB 8
#define WM8960_ALC_TARGET_LEVEL_NEG_9DB 9
#define WM8960_ALC_TARGET_LEVEL_NEG_7_5DB 10
#define WM8960_ALC_TARGET_LEVEL_NEG_6DB 11
#define WM8960_ALC_TARGET_LEVEL_NEG_4_5DB 12
#define WM8960_ALC_TARGET_LEVEL_NEG_3DB 13
#define WM8960_ALC_TARGET_LEVEL_NEG_1_5DB 14

// Automatic Level Control Max Gain Level dB
#define WM8960_ALC_MAX_GAIN_LEVEL_NEG_12DB 0
#define WM8960_ALC_MAX_GAIN_LEVEL_NEG_6DB 1
#define WM8960_ALC_MAX_GAIN_LEVEL_0DB 2
#define WM8960_ALC_MAX_GAIN_LEVEL_6DB 3
#define WM8960_ALC_MAX_GAIN_LEVEL_12DB 4
#define WM8960_ALC_MAX_GAIN_LEVEL_18DB 5
#define WM8960_ALC_MAX_GAIN_LEVEL_24DB 6
#define WM8960_ALC_MAX_GAIN_LEVEL_30DB 7

// Automatic Level Control Min Gain Level dB
#define WM8960_ALC_MIN_GAIN_LEVEL_NEG_17_25DB 0
#define WM8960_ALC_MIN_GAIN_LEVEL_NEG_11_25DB 1
#define WM8960_ALC_MIN_GAIN_LEVEL_NEG_5_25DB 2
#define WM8960_ALC_MIN_GAIN_LEVEL_0_75DB 3
#define WM8960_ALC_MIN_GAIN_LEVEL_6_75DB 4
#define WM8960_ALC_MIN_GAIN_LEVEL_12_75DB 5
#define WM8960_ALC_MIN_GAIN_LEVEL_18_75DB 6
#define WM8960_ALC_MIN_GAIN_LEVEL_24_75DB 7

// Automatic Level Control Hold Time (MS and SEC)
#define WM8960_ALC_HOLD_TIME_0MS 0
#define WM8960_ALC_HOLD_TIME_3MS 1
#define WM8960_ALC_HOLD_TIME_5MS 2
#define WM8960_ALC_HOLD_TIME_11MS 3
#define WM8960_ALC_HOLD_TIME_21MS 4
#define WM8960_ALC_HOLD_TIME_43MS 5
#define WM8960_ALC_HOLD_TIME_85MS 6
#define WM8960_ALC_HOLD_TIME_170MS 7
#define WM8960_ALC_HOLD_TIME_341MS 8
#define WM8960_ALC_HOLD_TIME_682MS 9
#define WM8960_ALC_HOLD_TIME_1365MS 10
#define WM8960_ALC_HOLD_TIME_3SEC 11
#define WM8960_ALC_HOLD_TIME_5SEC 12
#define WM8960_ALC_HOLD_TIME_10SEC 13
#define WM8960_ALC_HOLD_TIME_23SEC 14
#define WM8960_ALC_HOLD_TIME_44SEC 15

// Automatic Level Control Decay Time (MS and SEC)
#define WM8960_ALC_DECAY_TIME_24MS 0
#define WM8960_ALC_DECAY_TIME_48MS 1
#define WM8960_ALC_DECAY_TIME_96MS 2
#define WM8960_ALC_DECAY_TIME_192MS 3
#define WM8960_ALC_DECAY_TIME_384MS 4
#define WM8960_ALC_DECAY_TIME_768MS 5
#define WM8960_ALC_DECAY_TIME_1536MS 6
#define WM8960_ALC_DECAY_TIME_3SEC 7
#define WM8960_ALC_DECAY_TIME_6SEC 8
#define WM8960_ALC_DECAY_TIME_12SEC 9
#define WM8960_ALC_DECAY_TIME_24SEC 10

// Automatic Level Control Attack Time (MS and SEC)
#define WM8960_ALC_ATTACK_TIME_6MS 0
#define WM8960_ALC_ATTACK_TIME_12MS 1
#define WM8960_ALC_ATTACK_TIME_24MS 2
#define WM8960_ALC_ATTACK_TIME_482MS 3
#define WM8960_ALC_ATTACK_TIME_964MS 4
#define WM8960_ALC_ATTACK_TIME_1928MS 5
#define WM8960_ALC_ATTACK_TIME_3846MS 6
#define WM8960_ALC_ATTACK_TIME_768MS 7
#define WM8960_ALC_ATTACK_TIME_1536MS 8
#define WM8960_ALC_ATTACK_TIME_3SEC 9
#define WM8960_ALC_ATTACK_TIME_6SEC 10

// Speaker Boost Gains (DC and AC)
#define WM8960_SPEAKER_BOOST_GAIN_0DB 0
#define WM8960_SPEAKER_BOOST_GAIN_2_1DB 1
#define WM8960_SPEAKER_BOOST_GAIN_2_9DB 2
#define WM8960_SPEAKER_BOOST_GAIN_3_6DB 3
#define WM8960_SPEAKER_BOOST_GAIN_4_5DB 4
#define WM8960_SPEAKER_BOOST_GAIN_5_1DB 5

// VMIDSEL settings
#define WM8960_VMIDSEL_DISABLED 0
#define WM8960_VMIDSEL_2X50KOHM 1
#define WM8960_VMIDSEL_2X250KOHM 2
#define WM8960_VMIDSEL_2X5KOHM 3

// VREF to Analogue Output Resistance
// (Disabled Outputs)
// 0 = 500 VMID to output
// 1 = 20k VMID to output 
#define WM8960_VROI_500 0
#define WM8960_VROI_20K 1

// Analogue Bias Optimisation
// 00 = Reserved
// 01 = Increased bias current optimized for
// AVDD=2.7V
// 1X = Lowest bias current, optimized for
// AVDD=3.3V 

#define WM8960_VSEL_INCREASED_BIAS_CURRENT 1
#define WM8960_VSEL_LOWEST_BIAS_CURRENT 3

// JACK DETECT INPUT
#define WM8960_JACKDETECT_GPIO1 0
#define WM8960_JACKDETECT_LINPUT3 1
#define WM8960_JACKDETECT_RINPUT3 2

void WM8960Init( void );
void WM8960SetHeadphoneVolume( void );
void WM8960SetLeftADCVolume( uint8_t vol );
void WM8960SetRightADCVolume( uint8_t vol );
uint8_t WM8960GetLeftADCVolume( void );
uint8_t WM8960GetRightADCVolume( void );
void WM8960SetInputBoostGain( uint8_t gain );
uint8_t WM8960GetInputBoostGain( void );
void WM8960MuteInput();
void WM8960UnmuteInput();

#endif
