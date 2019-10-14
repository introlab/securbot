/**
 * @file defines.hpp
 * @author Cedric Godin
 * @brief Various definitions for whole firmware
 * @version 0.1
 * @date 2019-10-01
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#pragma once

/**
 * @brief Onboard LED GPIO.
 * GPIO number of the LED built in on the ESP32 Feather
 */
#define ONBOARD_LED_PIN 13

/**
 * @brief BQ24725A VCC enable GPIO.
 * GPIO number to U4 which apply power to the VCC pin
 */
#define BQ24725A_EN_GPIO 25

/**
 * @brief Relay CMD GPIO.
 * GPIO number that controls the relay providing power to the robot
 */
#define CMD_RELAY_GPIO 26

/**
 * @brief Fan CMD GPIO
 * GPIO number that controls the fan to cooldown the batterie
 */
#define CMD_FAN_GPIO 23

/**
 * @brief i2c 0 SDA pin.
 * i2c bus 0 SDA pin number
 */
#define I2C_NUM_0_SDA_PIN 17
/**
 * @brief i2c 0 SCL pin.
 * i2c bus 0 SCL pin number
 */
#define I2C_NUM_0_SCL_PIN 16
/**
 * @brief i2c 1 SDA pin.
 * i2c bus 1 SDA pin number
 */
#define I2C_NUM_1_SDA_PIN 5
/**
 * @brief i2c 1 SCL pin.
 * i2c bus 1 SCL pin number
 */
#define I2C_NUM_1_SCL_PIN 4

/**
 * @brief i2c clock speed.
 * i2c master clock speed. Maximum 1 Mhz.
 */
#define I2C_CLK_SPEED 400000
/**
 * @brief smbus clock speed.
 * smbus master clock speed. Maxumum 100 khz.
 */
#define SMBUS_CLK_SPEED 100000

/**
 * @brief ADS1015 i2c bus.
 * The i2c bus to which the ADS1015 chips are connected
 */
#define ADS1015_I2C_NUM I2C_NUM_0
/**
 * @brief BQ24725A i2c bus.
 * The i2c bus to which the BQ24725A chip is connected
 */
#define BQ24725A_I2C_NUM I2C_NUM_1

/**
 * @brief ADC0 i2c address.
 * i2c address of the ADS1015 to which analog channels 0-3 are connected
 */
#define ADC0_I2C_ADDRESS 0b1001001
/**
 * @brief ADC1 i2c address.
 * i2c address of the ADS1015 to which analog channels 4-7 are connected
 */
#define ADC1_I2C_ADDRESS 0b1001000

/**
 * @brief BMS VCOUT channel.
 * analog channel connected to BMS VCOUT
 */
#define VCOUT_BMS_CHANNEL 0
/**
 * @brief BMS VIOUT channel.
 * analog channel connected to BMS VIOUT
 */
#define VIOUT_BMS_CHANNEL 1
/**
 * @brief Thermistor R10 channel.
 * analog channel connected to Thermistor R10
 */
#define THERMISTOR_R10_CHANNEL 2
/**
 * @brief Thermistor R11 channel.
 * analog channel connected to thermistor R11
 */
#define THERMISTOR_R11_CHANNEL 3
/**
 * @brief BQ24725A IOUT channel.
 * analog channel connected to BQ24725A IOUT
 */
#define BQ24725A_IOUT_CHANNEL 4
