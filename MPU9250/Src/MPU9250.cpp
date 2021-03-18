// 
// \file MPU9250.h
// \author Lucas Meira
// \brief Declaration of base MPU9250 class.
// 

#include "MPU9250.h"

#include <string.h>
#include <memory>

namespace mpu9250
{
    MPU9250::MPU9250(SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPinPort, uint16_t csPin)
    {
        spiHandle_ = spiHandle;
        csPinPort_ = csPinPort;
        csPin_ = csPin;
    }

    bool MPU9250::init()
    {
        // Config clock speed must be lower than 1MHz.
        setSpiClockSpeed(1);

        // Activate chip master I²C mode and disable I²C communication to use SPI.
        ASSERT_SUCCESS(write(USER_CTRL, I2C_MST_EN | I2C_IF_DIS));

        ASSERT_SUCCESS(write(I2C_MST_CTRL, I2C_MST_CLK_400KHZ));

        ASSERT_SUCCESS(writeAk8963(AK8963_CNTL1, AK8693_PWRDWN));

        // Reset the MPU9250
        write(PWR_MGMNT_1, H_RESET);

        HAL_Delay(1);

        // Reset the AK8963
        writeAk8963(AK8963_CNTL2, AK8963_SRST);

        // Select PLL as a clock source for better accuracy.
        ASSERT_SUCCESS(write(PWR_MGMNT_1, CLKSEL_PLL));

        // Activate chip master I²C mode and disable I²C communication to use SPI.
        ASSERT_SUCCESS(write(USER_CTRL, I2C_MST_EN | I2C_IF_DIS));

        ASSERT_SUCCESS(write(I2C_MST_CTRL, I2C_MST_CLK_400KHZ));

        uint8_t whoAmI = 0;
        read(WHOAMI, &whoAmI, sizeof(whoAmI));

        if (whoAmI != WHOAMI_VAL)
        {
            return false;
        }

        ASSERT_SUCCESS(readAk8963(AK8963_WHOAMI, &whoAmI, sizeof(whoAmI)));

        if (whoAmI != AK8963_WHOAMI_VAL)
        {
            return false;
        }

        ASSERT_SUCCESS(initAk8963());

        setSpiClockSpeed(20);

        return true;
    }

    bool MPU9250::initAk8963()
    {
        ASSERT_SUCCESS(writeAk8963(AK8963_CNTL1, AK8963_FUSEROM));

        // Wait for AK8693 to change modes.
        HAL_Delay(1);

        uint8_t sensitivities[3];
        ASSERT_SUCCESS(readAk8963(AK8963_ASA, sensitivities, sizeof(sensitivities)));

        magScale_.x = (((sensitivities[0] - 128.0f) / 256.0f) + 1) * 4912.0f / 32760.0f;
        magScale_.y = (((sensitivities[1] - 128.0f) / 256.0f) + 1) * 4912.0f / 32760.0f;
        magScale_.z = (((sensitivities[2] - 128.0f) / 256.0f) + 1) * 4912.0f / 32760.0f;

        // Return to powerdown mode in order to change to other mode.
        ASSERT_SUCCESS(writeAk8963(AK8963_CNTL1, AK8693_PWRDWN));

        HAL_Delay(1);

        // Set AK8963 to 16 bit resolution, 100 Hz update rate.
        ASSERT_SUCCESS(writeAk8963(AK8963_CNTL1, AK8963_CNT_MEAS_2 | AK8963_OUTPUT_16_BITS));

        HAL_Delay(1);

        // Configure MPU to read magnetometer. Read 3 axes and status2 register (mandatory).
        uint8_t magData[7];
        ASSERT_SUCCESS(readAk8963(AK8963_HXL, magData, sizeof(magData)));

        return true;
    }

    bool MPU9250::get()
    {
        txBuff_[0] = ACCEL_XOUT_H | READ_FLAG;

        HAL_GPIO_WritePin(csPinPort_, csPin_, GPIO_PIN_RESET);

        if (HAL_SPI_TransmitReceive_DMA(spiHandle_, txBuff_, rxBuff_, READ_BUFFER_LEN) != HAL_OK)
        {
            return false;
        }

        return true;
    }

    void MPU9250::dmaReadComplete()
    {
        HAL_GPIO_WritePin(csPinPort_, csPin_, GPIO_PIN_SET);

        rawData.accel.x = rxBuff_[1] << 8 | rxBuff_[2];
        rawData.accel.y = rxBuff_[3] << 8 | rxBuff_[4];
        rawData.accel.z = rxBuff_[5] << 8 | rxBuff_[6];

        rawData.temperature = rxBuff_[7] << 8 | rxBuff_[8];

        rawData.gyro.x = rxBuff_[9] << 8 | rxBuff_[10];
        rawData.gyro.y = rxBuff_[11] << 8 | rxBuff_[12];
        rawData.gyro.z = rxBuff_[13] << 8 | rxBuff_[14];

        rawData.magnet.x = rxBuff_[16] << 8 | rxBuff_[15];
        rawData.magnet.y = rxBuff_[18] << 8 | rxBuff_[17];
        rawData.magnet.z = rxBuff_[20] << 8 | rxBuff_[19];

        convert();
    }

    void MPU9250::convert()
    {
        data.accel.x = rawData.accel.x * accScale_;
        data.accel.y = rawData.accel.y * accScale_;
        data.accel.z = rawData.accel.z * accScale_;

        data.gyro.x = rawData.gyro.x * gyroScale_;
        data.gyro.y = rawData.gyro.y * gyroScale_;
        data.gyro.z = rawData.gyro.z * gyroScale_;

        data.temperature = (rawData.temperature - 21.0f) / tempScale_ + 21.0f;

        data.magnet.x = rawData.magnet.x * magScale_.x;
        data.magnet.y = rawData.magnet.y * magScale_.y;
        data.magnet.z = rawData.magnet.z * magScale_.z;
    }

    bool MPU9250::read(const uint8_t regAddress, uint8_t *in, const uint8_t size)
    {
        uint8_t tx = READ_FLAG | regAddress;

        HAL_GPIO_WritePin(csPinPort_, csPin_, GPIO_PIN_RESET);

        if (HAL_SPI_Transmit(spiHandle_, (uint8_t*) &tx, sizeof(regAddress), 1000) != HAL_OK)
        {
            return false;
        }

        if (HAL_SPI_Receive(spiHandle_, in, size, 1000) != HAL_OK)
        {
            return false;
        }

        HAL_GPIO_WritePin(csPinPort_, csPin_, GPIO_PIN_SET);

        return true;
    }

    bool MPU9250::write(const uint8_t regAddress, const uint8_t out)
    {
        txBuff_[0] = regAddress;
        txBuff_[1] = out;

        HAL_GPIO_WritePin(csPinPort_, csPin_, GPIO_PIN_RESET);

        if (HAL_SPI_Transmit(spiHandle_, txBuff_, sizeof(regAddress) + sizeof(out), 1000) != HAL_OK)
        {
            return false;
        }

        HAL_GPIO_WritePin(csPinPort_, csPin_, GPIO_PIN_SET);

        return true;
    }

    bool MPU9250::readAk8963(const uint8_t regAddress, uint8_t *in, const uint8_t size)
    {
        ASSERT_SUCCESS(write(I2C_SLV0_ADDR, READ_FLAG | AK8963_I2C_ADDR));

        ASSERT_SUCCESS(write(I2C_SLV0_REG, regAddress));

        // Limit to 4 bits write
        ASSERT_SUCCESS(write(I2C_SLV0_CTRL, READ_FLAG | (size & 0x0F)));

        HAL_Delay(1);

        return read(EXT_SENS_DATA_00, in, size);
    }

    bool MPU9250::writeAk8963(const uint8_t regAddress, uint8_t out)
    {
        ASSERT_SUCCESS(write(I2C_SLV0_ADDR, AK8963_I2C_ADDR));

        ASSERT_SUCCESS(write(I2C_SLV0_REG, regAddress));

        ASSERT_SUCCESS(write(I2C_SLV0_DO, out));

        // Limit to 4 bits write
        ASSERT_SUCCESS(write(I2C_SLV0_CTRL, READ_FLAG | (sizeof(data) & 0x0F)));

        // Wait for the chips to communicate between themselves.
        HAL_Delay(1);

        uint8_t regVal = 0;
        ASSERT_SUCCESS(readAk8963(regAddress, &regVal, sizeof(regVal)));

        // Check if the value has been correctly written.
        if (regVal != out)
        {
            return false;
        }

        return true;
    }

    bool MPU9250::setInterruptMode(InterruptEnable intMode)
    {
        setSpiClockSpeed(1);

        // Configure any read to clear interrupt, active high, 50us pulse and open drain.
        ASSERT_SUCCESS(write(INT_PIN_CFG, INT_ANYRD_2CLEAR));

        ASSERT_SUCCESS(write(INT_ENABLE, intMode));

        setSpiClockSpeed(20);

        return true;
    }

    void MPU9250::setSpiClockSpeed(uint16_t speed)
    {
        if (speed > 20)
        {
            speed = 20;
        }
        else if (speed == 0)
        {
            speed = 1;
        }

        uint32_t prescaler = SPI_BAUDRATEPRESCALER_2;

        RCC_ClkInitTypeDef RCC_ClkInitStruct;
        uint32_t temp;

        HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &temp);

        uint16_t apb2ClkDivider = 0;

        switch (RCC_ClkInitStruct.APB2CLKDivider)
        {
            case RCC_CFGR_PPRE1_DIV1:
                apb2ClkDivider = 1;
                break;
            case RCC_CFGR_PPRE1_DIV2:
                apb2ClkDivider = 2;
                break;
            case RCC_CFGR_PPRE1_DIV4:
                apb2ClkDivider = 4;
                break;
            case RCC_CFGR_PPRE1_DIV8:
                apb2ClkDivider = 8;
                break;
            case RCC_CFGR_PPRE1_DIV16:
                apb2ClkDivider = 16;
                break;
            default:
                apb2ClkDivider = 1;
        }

        uint32_t spiClkFreq = SystemCoreClock / apb2ClkDivider / 1000; // Freq in kHz

        uint32_t tempPrescaler = spiClkFreq / (speed * 1000);

        if (tempPrescaler <= 2)
        {
            prescaler = SPI_BAUDRATEPRESCALER_2;
        }
        else if (tempPrescaler <= 4)
        {
            prescaler = SPI_BAUDRATEPRESCALER_4;
        }
        else if (tempPrescaler <= 8)
        {
            prescaler = SPI_BAUDRATEPRESCALER_8;
        }
        else if (tempPrescaler <= 16)
        {
            prescaler = SPI_BAUDRATEPRESCALER_16;
        }
        else if (tempPrescaler <= 32)
        {
            prescaler = SPI_BAUDRATEPRESCALER_32;
        }
        else if (tempPrescaler <= 64)
        {
            prescaler = SPI_BAUDRATEPRESCALER_64;
        }
        else if (tempPrescaler <= 128)
        {
            prescaler = SPI_BAUDRATEPRESCALER_128;
        }
        else if (tempPrescaler <= 256)
        {
            prescaler = SPI_BAUDRATEPRESCALER_256;
        }

        spiHandle_->Instance->CR1 &= ~SPI_CR1_BR_Msk;

        spiHandle_->Instance->CR1 |= prescaler;
    }
}
