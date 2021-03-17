/**
 * \file MPU9250.h
 * \author Lucas Meira
 * \brief Declaration of base MPU9250 class.
 */

#ifndef MPU9250_H
#define MPU9250_H

#include "spi.h"
#include "gpio.h"
#include "imu.h"
#include "MPU9250_definitions.h"

#define ASSERT_SUCCESS(rc) {if (!rc) return false;}

namespace mpu9250
{

	using namespace imu;

    struct MPU9250_s
    {
        raw::Accelerometer accel;
        uint16_t temperature;
        raw::Gyroscope gyro;
        raw::Magnetometer magnet;
    } __attribute__((packed));

    struct Converted
    {
        si::Accelerometer accel;
        float temperature;
        si::Gyroscope gyro;
        si::Magnetometer magnet;
    };

    class MPU9250
    {
    public:
        MPU9250(SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPinPort, uint16_t csPin);
        ~MPU9250() = default;

        bool init();
        bool get();

        void convert();

        bool read(const uint8_t regAddress, uint8_t *in, const uint8_t size);
        bool write(const uint8_t regAddress, const uint8_t out);

        bool readAk8963(const uint8_t regAddress, uint8_t *in, const uint8_t size);
        bool writeAk8963(const uint8_t regAddress, uint8_t out);

        bool setInterruptMode(InterruptEnable intMode);

        void dmaReadComplete();

        MPU9250_s rawData;
        Converted data;

    private:
        bool initAk8963();

        SPI_HandleTypeDef *spiHandle_;
        GPIO_TypeDef *csPinPort_;
        uint16_t csPin_;

        uint8_t txBuff_[READ_BUFFER_LEN];
		uint8_t rxBuff_[READ_BUFFER_LEN];

        imu::si::Magnetometer magScale_;
        float accScale_ = 9.81f / 16384.0f;
        float gyroScale_ = 1 / 131.0f;
        float tempScale_ = 333.87f;
    };
}
#endif
