/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#pragma once
#include <drivers/spi/SPIDriver.h>
#include <sensors/Sensor.h>
#include <sensors/SensorData.h>
#include <diagnostic/PrintLogger.h>

/**
 * @brief max6675 temperature sensor driver
 */
class MAX6675 : public Sensor<TemperatureData>
{
private:

    /**
     * @brief SPI slave object of the sensor
     */
    SPISlave slave;

    /**
     * @brief Initialization flag
     */
    bool isInit;

    /**
     * @brief SampleImplementation method used in sample method (Sensor class)
     * 
     * @return The temperature data
     */
    TemperatureData sampleImpl() override;

public:

    /**
     * @brief Constructor
     * 
     * @param The SPI bus 
     * @param The CS pin to lower when we need to sample
     */
    MAX6675(SPIBusInterface &bus, GpioPin cs);

    /**
     * @brief Constructor
     * 
     * @param The Spi bus
     * @param The CS pin to lower when we need to sample
     * @param The SPI configuration
     */
    MAX6675(SPIBusInterface &bus, GpioPin cs, SPIBusConfig config);

    /**
     * @brief Init method to initialize the stuff that we need to communicate
     * with the sensor. (E.g SPI communication)
     * 
     * @return The init process result (boolean value)
     */
    bool init();

    /**
     * @brief Self test method to ensure that the sensor is up and running
     * 
     * @return The self test result (boolean value)
     */
    bool selfTest();
};