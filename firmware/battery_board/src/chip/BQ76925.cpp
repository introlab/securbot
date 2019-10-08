/**
 * @file BQ76925.cpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Driver for the BQ76925 analog front end chip
 * @version 0.1
 * @date 2019-10-08
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "chip/BQ76925.hpp"

#define STATUS_REG_ADR 0    // STATUS register address
#define CELL_CTL_REG_ADR 1  // cell ctl register address
#define BAL_CTL_REG_ADR 2   // bal ctl register address
#define CONFIG_1_REG_ADR 3  // config 1 register address
#define CONFIG_2_REG_ADR 4  // config 2 register address
#define POWER_CTL_REG_ADR 5 // power ctl register address