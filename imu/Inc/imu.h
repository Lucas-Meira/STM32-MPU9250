/**
 * \file imu.h
 * \author Lucas Meira
 * \brief Declaration of base IMU types.
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>

namespace imu
{
    namespace raw
    {
        /**
         * \struct Accelerometer
         * \details Contains raw data for the three axes of the accelerometer.
         */
        struct Accelerometer
        {
            int16_t x;
            int16_t y;
            int16_t z;
        } __attribute__((packed));

        /**
         * \struct Gyroscope
         * \details Contains raw data for the three axes of the Gyroscope.
         */
        struct Gyroscope
        {
            int16_t x;
            int16_t y;
            int16_t z;
        } __attribute__((packed));

        /**
         * \struct Magnetometer
         * \details Contains raw data for the three axes of the Magnetometer.
         */
        struct Magnetometer
        {
            int16_t x;
            int16_t y;
            int16_t z;
        } __attribute__((packed));
    }

    namespace si
    {
        /**
         * \struct Accelerometer
         * \details Contains raw data for the three axes of the accelerometer.
         */
        struct Accelerometer
        {
            float x;
            float y;
            float z;
        };

        /**
         * \struct Gyroscope
         * \details Contains raw data for the three axes of the Gyroscope.
         */
        struct Gyroscope
        {
            float x;
            float y;
            float z;
        };

        /**
         * \struct Magnetometer
         * \details Contains raw data for the three axes of the Magnetometer.
         */
        struct Magnetometer
        {
            float x;
            float y;
            float z;
        };
    }

}

#endif
