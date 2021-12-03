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

#include "MAX6675.h"

/**
 * CONSTRUCTORS
 * ---------------------------------------------------------------------------------------------------------------------------------------
 */
MAX6675::MAX6675(SPIBusInterface &bus, GpioPin cs)
        :MAX6675(bus, cs, SPIBusConfig{})
{
    //Setting the SPI baud rate because no config object has been provided
    slave.config.clock_div = SPIClockDivider::DIV32;
    slave.config.mode = SPIMode::MODE1;
}

MAX6675::MAX6675(SPIBusInterface &bus, GpioPin cs, SPIBusConfig config)
        :slave(bus, cs, config)
{
    isInit = false;
}

/**
 * PUBLIC METHODS
 * ---------------------------------------------------------------------------------------------------------------------------------------
 */
bool MAX6675::init()
{
    //Initialize the transaction that NEEDS to be initialized
    //inside every method.
    SPITransaction spi{slave};

    if(isInit)
    {
        //Sensor already init
        last_error = SensorErrors::ALREADY_INIT;
        LOG_WARN(logger, "Sensor max6675 already initialized");
        return true;
    }

    //It doesn't matter what i send because the sensor is connected only on miso
    uint16_t sample = spi.read(0x00);

    if(sample == 0)
    {
        last_error = SensorErrors::NOT_INIT;
        LOG_ERR(logger, "Sensor max6675 not up and running");
        return false;
    }

    isInit = true;
    return true;
}

bool MAX6675::selfTest()
{
    //Initialize the transaction that NEEDS to be initialized
    //inside every method.
    SPITransaction spi{slave};

    //In case not initialized the self test fails
    if(!isInit)
    {
        last_error = SensorErrors::NOT_INIT;
        LOG_WARN(logger, "Sensor max6675 not initialized");
        return false;
    }

    //I sample the sensor and if the second bit and the 16th (little endian) are 0 
    //the sensor is up and running (the 16th bit is the dummy sign and the second is device ID)
    uint16_t sample = spi.read(0x00);

    if((sample & 0x8000) != 0x0000)
    {
        //The 16th bit is 1 so failed self test
        last_error = SensorErrors::SELF_TEST_FAIL;
        LOG_ERR(logger, "Sensor max6675 selft test fail: 16th bit is 1");
        return false;
    }

    if((sample & 0x0002) != 0x0000)
    {
        //The second bit is 1 so failed self test
        last_error = SensorErrors::SELF_TEST_FAIL;
        LOG_ERR(logger, "Sensor max6675 selft test fail: 2nd bit is 1");
        return false;
    }
    return true;
}

/**
 * PRIVATE METHODS
 * ---------------------------------------------------------------------------------------------------------------------------------------
 */
TemperatureData MAX6675::sampleImpl()
{
    //Initialize the transaction that NEEDS to be initialized
    //inside every method.
    SPITransaction spi{slave};

    //Result variable
    TemperatureData result{};

    //In case not initialized i return the last sample
    if(!isInit)
    {
        last_error = SensorErrors::NOT_INIT;
        LOG_WARN(logger, "Sensor max6675 not initialized");
        return last_sample;
    }

    //Sample the sensor and take the bit 14-3 (little endian)
    uint16_t sample = spi.read(0x00);
    //I isolate the sampling from other stuff
    sample = sample & 0x7FF8;
    //I shift the value
    sample = sample >> 3;

    //Result assign
    result.temp_timestamp   = TimestampTimer::getTimestamp();
    result.temp             = (unsigned int) sample >> 2;                   //Integer part
    
    //Take the floating point part
    sample = sample & 0x0003;
    result.temp            += (float)(sample * 0.25);                       //Floating point part

    return result;
}