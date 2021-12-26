/* Copyright (c) 2018-2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <miosix.h>
#include <sensors/Sensor.h>
#include <stdint.h>
#include <utils/Debug.h>

#include "AD7994Data.h"

/**
 * Driver for the AD7994 Analog Digital Converter
 * CLASS INVARIANT: The address pointer register points to the conversion result
 * register.
 * This is done for performance reasons: we don't want to write to the
 * address pointer register each time we perform a conversion (eg multiple times
 * per second)
 */
template <typename BusI2C, typename BusyPin, typename CONVST>
class AD7994 : public Sensor
{
public:
    enum class Channel : uint8_t
    {
        CH1 = 1,
        CH2,
        CH3,
        CH4
    };

    /**
     * @brief AD7994 constructor. Invariant is respected as the sensor powers up
     * pointing at the conversion result register.
     *
     * @param i2c_address I2C address of the AD7994
     */
    AD7994(uint8_t i2c_address)
        : i2c_address(i2c_address), enabled_channels{0, 0, 0, 0}
    {
    }

    ~AD7994() {}

    /**
     * @brief Initialize the sensor writing the configuration register
     * This driver is designed to work in Mode 1
     * The configuration register is set to enable the BUSY pin, I2C filtering
     * and no enabled channels.
     * @return true If the sensor was configured successfully
     */
    bool init() override
    {
        uint8_t config_reg_value = 0x0A;  // 0b00001010

        // Write the configuration register
        BusI2C::write(i2c_address, REG_CONFIG, &config_reg_value, 1);

        uint8_t read_config_reg_value;

        // Read back the value
        BusI2C::directRead(i2c_address, &read_config_reg_value, 1);
        // BusI2C::read(i2c_address, REG_CONFIG, &read_config_reg_value, 1);

        pointToConversionResult();

        return read_config_reg_value == config_reg_value;
    }

    void enableChannel(Channel channel)
    {
        uint8_t ch               = static_cast<uint8_t>(channel);
        enabled_channels[ch - 1] = true;

        uint8_t config_reg_value;
        BusI2C::read(i2c_address, REG_CONFIG, &config_reg_value, 1);

        // Update the config register value
        uint8_t channel_reg = 0;
        for (int i = 0; i < 4; i++)
        {
            if (enabled_channels[i])
                channel_reg |= 1 << i;
        }
        config_reg_value = (config_reg_value & 0x0F) | channel_reg << 4;

        BusI2C::write(i2c_address, REG_CONFIG, &config_reg_value, 1);

        pointToConversionResult();
    }

    void disableChannel(Channel channel)
    {
        uint8_t ch               = static_cast<uint8_t>(channel);
        enabled_channels[ch - 1] = false;

        uint8_t channel_reg = 0;
        for (int i = 0; i < 4; i++)
        {
            if (enabled_channels[i])
                channel_reg |= 1 << i;
        }

        uint8_t config_reg_value;
        BusI2C::read(i2c_address, REG_CONFIG, &config_reg_value, 1);

        // Update the config register value
        config_reg_value = (config_reg_value & 0x0F) | channel_reg << 4;

        BusI2C::write(i2c_address, REG_CONFIG, &config_reg_value, 1);

        pointToConversionResult();
    }

    /**
     * @brief Triggers a new ADC conversion on the enabled channels and reads
     * the value of the conversion register.
     * TODO: Check how to sample multiple channels.
     * @return true If the conversion was successful on all enabled channels
     */
    bool onSimpleUpdate() override
    {
        uint8_t data[2];

        for (int i = 0; i < 4; i++)
        {
            if (enabled_channels[i])
            {
                // Trigger a conversion
                CONVST::high();
                miosix::delayUs(3);
                CONVST::low();
                // Wait for the conversion to complete
                miosix::delayUs(2);

                BusI2C::directRead(i2c_address, data, 2);

                samples[i]           = decodeConversion(data);
                samples[i].timestamp = miosix::getTick();
            }
        }

        return true;
    }

    bool selfTest() { return true; }

    AD7994Sample getLastSample(Channel channel)
    {
        uint8_t ch = static_cast<uint8_t>(channel);
        return samples[ch - 1];
    }

private:
    // Address of the AD7994 on the I2C bus
    const uint8_t i2c_address;

    // The 4 LSBs indicate where the corresponding channel is enabled or not.
    bool enabled_channels[4];

    /**
     * @brief Writes the conversion register address to the address pointer
     * register in order to restore the class invariant.
     */
    void pointToConversionResult()
    {
        uint8_t reg_addr = REG_CONVERSION_RESULT;
        BusI2C::directWrite(i2c_address, &reg_addr, 1);
    }

    /**
     * @brief Decodes the 2 bytes of the conversion register into an
     * AD7994Sample structure
     *
     * @param data_ptr Pointer to the first of the 2 bytes of the conversione
     * register
     * @return AD7994Sample
     */
    AD7994Sample decodeConversion(uint8_t* data_ptr)
    {
        uint16_t conv_reg =
            static_cast<uint16_t>((data_ptr[0]) << 8) + data_ptr[1];

        AD7994Sample out;
        out.value      = conv_reg & 0x0FFF;
        out.channel_id = (static_cast<uint8_t>(conv_reg >> 12) & 0x03) + 1;
        out.alert_flag = static_cast<bool>(conv_reg >> 15);

        return out;
    }

    enum Registers : uint8_t
    {
        REG_CONVERSION_RESULT = 0x00,
        REG_ALERT_STATUS      = 0x01,
        REG_CONFIG            = 0x02,
    };

    AD7994Sample samples[4];
};
