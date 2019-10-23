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
 * @brief BQ76925 alert GPIO.
 * GPIO connected to the BQ76925 overcurrent alert pin
 */
#define BQ76925PWR_ALERT_GPIO 22

/**
 * @brief BQ76925 chip id.
 * Id that sould be returned when the chip id is red
 */
#define BQ76925_CHIP_ID 0x10

/**
 * @brief BQ24725A VCC enable GPIO.
 * GPIO number to U4 which apply power to the VCC pin
 */
#define BQ24725A_EN_GPIO 25

/**
 * @brief BQ24725A ACOK GPIO.
 * GPIO number to BQ24725A which indicate if the external power adapter is present
 */
#define BQ24725A_ACOK_GPIO 18

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
#define I2C_CLK_SPEED 100000
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

/**
 * @brief battery monitoring rate.
 * configures the rate at which the firmware reads the battery board state
 */
#define MONITOR_PERIOD_MS 1000

/**
 * @brief Maximum adapter current.
 * Sets the maximum adapter current *128mA = 8.064A
 */
#define ADAPTER_CURRENT 63

/**
 * @brief Battery final charge voltage.
 * Battery voltage at end of charge *16mV = 16.8V
 */
#define CHARGE_VOLTAGE 1050

/**
 * @brief Battery main charge current.
 * Battery current during main charge *64mA = 3.392A
 */
#define CHARGE_CURRENT 53

/**
 * @brief Battery balancing charge current.
 * Battery current during balance charge *64mA = 0.64A
 */
#define BALANCE_CURRENT 10

/**
 * @brief Min cell voltage
 * Minimum single battery cell voltage
 */
#define VMIN 2.0

/**
 * @brief Max cell voltage.
 * Maximum single battery cell voltage
 */
#define VMAX 4.2

/**
 * @brief Max safe cell delta.
 * Maximum delta between highest and lowest cell considered safe
 */
#define VDELTA_MAX 0.7

/**
 * @brief Voltage safety margin.
 * Margin applied to voltage safety check
 */
#define VMARGE 0.1

/**
 * @brief Max cell temperature.
 * Maximum safe temperature reading
 */
#define TMAX 60.0