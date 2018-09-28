/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef SRC_SHARED_DRIVERS_ADC_AD7994_H
#define SRC_SHARED_DRIVERS_ADC_AD7994_H

#include <miosix.h>
#include <stdint.h>

#include "Singleton.h"
#include "sensors/Sensor.h"

/**
 * Struct containing the data from each ADC channel
 */
struct ADCData
{
    uint16_t val_ch1;
    uint16_t val_ch2;
    uint16_t val_ch3;
    uint16_t val_ch4;
};

/**
 * Driver for the AD7994 Analog Digital Converter
 */
template <typename BusI2C>
class AD7994 : public Sensor
{
public:
    /**
     *
     * @param i2c_address address of the AD7994 on the I2C bus
     */
    AD7994(uint8_t i2c_address) : address(i2c_address) {}

    virtual ~AD7994() {}

    /**
     * Enables the specified ADC channel
     * @param channel Channel number [1-4]
     */
    void enableChannel(uint8_t channel)
    {
        if (channel >= 1 && channel <= 4)
        {
            uint8_t mask = 1 << (channel - 1);

            // If this channel is not yet selected
            if (selectedChannels & mask == 0)
            {
                // Select it and increase the counter
                selectedChannels |= mask;
                selectedChannelsNum++;
            }
        }
    }

    /**
    * Disables the specified ADC channel
    * @param channel Channel number [1-4]
    */
    void disableChannel(uint8_t channel)
    {
        if (channel >= 1 && channel <= 4)
        {
            uint8_t mask = ~(1 << (channel - 1));
            // If this channel is already selected
            if (selectedChannels & mask > 0)
            {
                // Deselect it and decrease the counter
                selectedChannels &= mask;
                selectedChannelsNum--;
            }
        }
    }

    void disableAllChannels()
    {
        selectedChannels    = 0;
        selectedChannelsNum = 0;
    }

    /**
     * Returns a pointer to the latest samples
     * @return
     */
    ADCData* adcDataPtr() { return &samples; }

    bool onSimpleUpdate()
    {
        // 1: Program address register to start conversion
        // TODO: Check if we have to write these bits beforehand
        uint8_t data = selectedChannels << 4;
        BusI2C::directWrite(address, &data, 1);
        // TODO: Read data
        return true;
    }

    bool selfTest() { return true; }

private:
    // Address of the AD7994 on the I2C bus
    uint8_t address;

    uint8_t selectedChannelsNum = 0;    // Number of selected channels
    uint8_t selectedChannels    = 0x0;  // The 4 most significant bits in the
                                        // address pointer register

    ADCData samples;

    enum Registers : uint8_t
    {
        CONVERSION_RESULT_REG = 0x00,
        ALERT_STATUS_REG      = 0x01,
        CONFIG_REG            = 0x02
    };
};

#endif /* SRC_SHARED_DRIVERS_ADC_AD7994_H */
