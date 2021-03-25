/**
 * \file MPU9250_definitions.h
 * \author Lucas Meira
 * \brief Definitions of register addresses of the MPU9250.
 */

#ifndef MPU9250_DEFINITIONS
#define MPU9250_DEFINITIONS

#include <cstdint>

namespace mpu9250
{
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

    const uint8_t ACTL = 0x80;
    const uint8_t OPEN = 0x40;
    const uint8_t LATCH_INT_EN = 0x20;
    const uint8_t INT_ANYRD_2CLEAR = 0x10;

    /* Enums and Constants */

    enum InterruptEnable : uint8_t
    {
        RAW_RDY_EN = 0x01,
        FSYNC_INT_EN = 0x08,
        FIFO_OVERFLOW_EN = 0x10,
        WOM_EN = 0x40
    };

    enum AccelRange : uint8_t
    {
        ACCEL_RANGE_2G = 0x00,
        ACCEL_RANGE_4G = 0x08,
        ACCEL_RANGE_8G = 0x10,
        ACCEL_RANGE_16G = 0x18,
        ACCEL_RANGE_DEFAULT = ACCEL_RANGE_2G
    };

    enum GyroRange : uint8_t
    {
        GYRO_RANGE_250DPS = 0x00,
        GYRO_RANGE_500DPS = 0x08,
        GYRO_RANGE_1000DPS = 0x10,
        GYRO_RANGE_2000DPS = 0x18,
        GYRO_RANGE_DEFAULT = GYRO_RANGE_250DPS
    };


    const float ACCEL_SCALE[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};

    const float ACCEL_SCALE_DEFAULT = ACCEL_SCALE[0];

    const float GYRO_SCALE[4] = {131.0f, 65.5f, 32.8f, 16.4f};

    const float GYRO_SCALE_DEFAULT = GYRO_SCALE[0];

    const uint16_t SPI_MAX_READ_FREQ = 20000;
    const uint16_t SPI_MAX_WRITE_FREQ = 1000;

    /* AK8963 registers */

    const uint8_t AK8963_WHOAMI = 0x00;
    const uint8_t AK8963_I2C_ADDR = 0x0C;
    const uint8_t AK8963_HXL = 0x03;
    const uint8_t AK8963_CNTL1 = 0x0A;
    const uint8_t AK8963_CNTL2 = 0x0B;
    const uint8_t AK8963_ASA = 0x10;

    // AKA8963 flags

    const uint8_t AK8693_PWRDWN = 0x00;
    const uint8_t AK8963_CNT_MEAS_2 = 0x06;
    const uint8_t AK8963_FUSEROM = 0x07;
    const uint8_t AK8963_OUTPUT_16_BITS = 0x10;
    const uint8_t AK8963_WHOAMI_VAL = 0x48;
    const uint8_t AK8963_SRST = 0x01;
}

#endif
