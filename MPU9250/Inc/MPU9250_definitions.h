/**
 * \file MPU9250_definitions.h
 * \author Lucas Meira
 * \brief Definitions of register addresses of the MPU9250.
 */

#ifndef MPU9250_DEFINITIONS
#define MPU9250_DEFINITIONS

#include <stdint.h>

const uint8_t WRITE_BUFFER_LEN = 2;
const uint8_t READ_BUFFER_LEN = 22; //1 byte + accel, gyro, temp, and magnetometer data
const uint8_t READ_FLAG = 0x80;


/* MPU9250 registers */
const uint8_t CONFIG = 0x1A;
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t ACCEL_CONFIG_2 = 0x1D;
const uint8_t LP_ACCEL_ODR = 0x1E;

const uint8_t I2C_MST_CTRL = 0x24;
const uint8_t I2C_SLV0_ADDR = 0x25;
const uint8_t I2C_SLV0_REG = 0x26;
const uint8_t I2C_SLV0_CTRL = 0x27;

const uint8_t INT_PIN_CFG = 0x37;
const uint8_t INT_ENABLE = 0x38;
const uint8_t INT_STATUS = 0x3A;

const uint8_t ACCEL_XOUT_H = 0x3B;
const uint8_t EXT_SENS_DATA_00 = 0x49;

const uint8_t I2C_SLV0_DO = 0x63;

const uint8_t USER_CTRL = 0x6A;
const uint8_t PWR_MGMNT_1 = 0x6B;
const uint8_t WHOAMI = 0x75;


/* MPU9250 flags */

const uint8_t I2C_MST_CLK_400KHZ = 0x0D;

const uint8_t I2C_MST_EN = 0b00100000;
const uint8_t I2C_IF_DIS = 0b00010000;

const uint8_t WHOAMI_VAL = 0x71;

const uint8_t CLKSEL_PLL = 0x01;

const uint8_t H_RESET = 0x80;

/* AK8963 registers */

const uint8_t AK8963_WHOAMI = 0x00;
const uint8_t AK8963_I2C_ADDR = 0x0C;
const uint8_t AK8963_HXL = 0x03;
const uint8_t AK8963_CNTL1 = 0x0A;
const uint8_t AK8963_CNTL2 = 0x0B;
const uint8_t AK8963_ASA = 0x10;

/* AKA8963 flags */

const uint8_t AK8693_PWRDWN = 0x00;
const uint8_t AK8963_CNT_MEAS_2 = 0x06;
const uint8_t AK8963_FUSEROM = 0x07;
const uint8_t AK8963_OUTPUT_16_BITS = 0x10;
const uint8_t AK8963_WHOAMI_VAL = 0x48;
const uint8_t AK8963_SRST = 0x01;

#endif
