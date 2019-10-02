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

// General definitions

/**
 * @brief Onboard LED GPIO
 * GPIO number of the LED built in on the ESP32 Feather
 */
#define ONBOARD_LED_PIN 13

// I2C definitions

/**
 * @brief i2c 0 SDA pin
 * i2c bus 0 SDA pin number
 */
#define I2C_NUM_0_SDA_PIN 17
/**
 * @brief i2c 0 SCL pin
 * i2c bus 0 SCL pin number
 */
#define I2C_NUM_0_SCL_PIN 16
/**
 * @brief i2c 1 SDA pin
 * i2c bus 1 SDA pin number
 */
#define I2C_NUM_1_SDA_PIN 5
/**
 * @brief i2c 1 SCL pin
 * i2c bus 1 SCL pin number
 */
#define I2C_NUM_1_SCL_PIN 4

/**
 * @brief i2c clock speed
 * i2c master clock speed. Maximum 1 Mhz.
 */
#define I2C_CLK_SPEED 400000

// ADC definitions

/**
 * @brief ADS1015 i2c bus
 * The i2c bus to which the ADS1015 chips are connected
 */
#define ADS1015_I2C_NUM I2C_NUM_0

// BQ24725A definitions

/**
 * @brief BQ24725A i2c bus
 * The i2c bus to which the BQ24725A chip is connected
 */
#define BQ24725A_I2C_NUM I2C_NUM_1