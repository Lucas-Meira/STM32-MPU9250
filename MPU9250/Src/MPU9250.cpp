/**
 * \file MPU9250.h
 * \author Lucas Meira
 * \brief Declaration of base MPU9250 class.
 */

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
        /* Activate chip master I²C mode and disable I²C communication to use SPI. */
        if (!write(USER_CTRL, I2C_MST_EN))
        {
            return false;
        }

        uint8_t whoAmI = 0;
        read(WHOAMI, &whoAmI, sizeof(whoAmI));

        if (whoAmI != WHOAMI_VAL)
        {
            return false;
        }

        if (!write(I2C_MST_CTRL, I2C_MST_CLK_400KHZ))
        {
            return false;
        }

        HAL_Delay(1);

        if (!readAk8963(AK8963_WHOAMI, &whoAmI, sizeof(whoAmI)))
		{
			return false;
		}

		if (whoAmI != AK8963_WHOAMI_VAL)
		{
			return false;
		}

        if (!writeAk8963(AK8963_CNTL1, POWERDOWN_MODE))
        {
            return false;
        }

        HAL_Delay(1);

        // Set AK8963 to 16 bit resolution, 100 Hz update rate
        if (!writeAk8963(AK8963_CNTL1, AK8963_CNT_MEAS_2 | AK8963_OUTPUT_16_BITS))
        {
            return false;
        }

        HAL_Delay(1);

        // Configure MPU to read magnetometer
        uint8_t magData[7];
        if (!readAk8963(AK8963_HXL, magData, sizeof(magData)))
        {
            return false;
        }

        return true;
    }

    bool MPU9250::get()
    {
        txBuff[0] = ACCEL_XOUT_H | READ_FLAG;

        HAL_GPIO_WritePin(csPinPort_, csPin_, GPIO_PIN_RESET);

        if (HAL_SPI_TransmitReceive_DMA(spiHandle_, txBuff, rxBuff, READ_BUFFER_LEN) != HAL_OK)
        {
            return false;
        }

        return true;
    }

    void MPU9250::dmaReadComplete()
    {
        HAL_GPIO_WritePin(csPinPort_, csPin_, GPIO_PIN_SET);

        rawData.accel.x = rxBuff[1] << 8 | rxBuff[2];
        rawData.accel.y = rxBuff[3] << 8 | rxBuff[4];
        rawData.accel.z = rxBuff[5] << 8 | rxBuff[6];

        rawData.temperature = rxBuff[7] << 8 | rxBuff[8];

		rawData.gyro.x = rxBuff[9] << 8 | rxBuff[10];
		rawData.gyro.y = rxBuff[11] << 8 | rxBuff[12];
		rawData.gyro.z = rxBuff[13] << 8 | rxBuff[14];

		rawData.magnet.x = rxBuff[16] << 8 | rxBuff[15];
		rawData.magnet.y = rxBuff[18] << 8 | rxBuff[17];
		rawData.magnet.z = rxBuff[20] << 8 | rxBuff[19];

        convert();
    }

    void MPU9250::convert()
    {
        data.accel.x = rawData.accel.x * accScale;
        data.accel.y = rawData.accel.y * accScale;
        data.accel.z = rawData.accel.z * accScale;

        data.gyro.x = rawData.gyro.x * gyroScale;
        data.gyro.y = rawData.gyro.y * gyroScale;
        data.gyro.z = rawData.gyro.z * gyroScale;

        data.temperature = (rawData.temperature - 21.0f) / tempScale + 21.0f;

        data.magnet.x = rawData.magnet.x * magScale;
        data.magnet.y = rawData.magnet.y * magScale;
        data.magnet.z = rawData.magnet.z * magScale;
    }

    bool MPU9250::read(const uint8_t regAddress, uint8_t *in, const uint8_t size)
    {
    	uint8_t txBuff = READ_FLAG | regAddress;

        HAL_GPIO_WritePin(csPinPort_, csPin_, GPIO_PIN_RESET);

        if (HAL_SPI_Transmit(spiHandle_, (uint8_t *) &txBuff, sizeof(regAddress), 1000) != HAL_OK)
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
        uint8_t txBuff[WRITE_BUFFER_LEN];

        txBuff[0] = regAddress;
        txBuff[1] = out;

        HAL_GPIO_WritePin(csPinPort_, csPin_, GPIO_PIN_RESET);

        if (HAL_SPI_Transmit(spiHandle_, txBuff, sizeof(txBuff), 1000) != HAL_OK)
        {
            return false;
        }

        HAL_GPIO_WritePin(csPinPort_, csPin_, GPIO_PIN_SET);

        return true;
    }

    bool MPU9250::readAk8963(const uint8_t regAddress, uint8_t *in, const uint8_t size)
    {
        if (!write(I2C_SLV0_ADDR, READ_FLAG | AK8963_I2C_ADDR))
        {
            return false;
        }

        if (!write(I2C_SLV0_REG, regAddress))
        {
            return false;
        }

        // Limit to 4 bits write
        if (!write(I2C_SLV0_CTRL, READ_FLAG | (size & 0x0F)))
        {
            return false;
        }

        HAL_Delay(1);

        return read(EXT_SENS_DATA_00, in, size);
    }

    bool MPU9250::writeAk8963(const uint8_t regAddress, uint8_t out)
    {
        if (!write(I2C_SLV0_ADDR, AK8963_I2C_ADDR))
        {
            return false;
        }

        if (!write(I2C_SLV0_REG, regAddress))
        {
            return false;
        }

        if (!write(I2C_SLV0_DO, out))
        {
            return false;
        }

        // Limit to 4 bits write
        if (!write(I2C_SLV0_CTRL, READ_FLAG | (sizeof(data) & 0x0F)))
        {
            return false;
        }

        HAL_Delay(1);

        uint8_t regVal = 0;
        if (!readAk8963(regAddress, &regVal, sizeof(regVal)))
        {
            return false;
        }

        if (regVal != out)
        {
        	return false;
        }

        return true;
    }
}
