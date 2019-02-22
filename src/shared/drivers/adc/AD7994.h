/* Copyright (c) 2018-2019 Skyward Experimental Rocketry
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

#include "sensors/Sensor.h"

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
        CH1,
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
    AD7994(uint8_t i2c_address) : i2c_address(i2c_address)
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
        const uint8_t config_reg_value = 0b00001010;

        // Write the configuration register
        BusI2c::write(i2c_address, REG_CONFIG, &config_reg_value, 1);

        uint8_t read_config_reg_value;

        // Read back the value
        BusI2c::read(i2c_address, REG_CONFIG, &read_config_reg_value, 1);

        pointToConversionResult();

        return read_config_reg_value == config_reg_value;
    }

    void enableChannel(Channel channel)
    {
        uint8_t channel = static_cast<uint8_t> channel;
        enabled_channels |= 1 << channel;

        uint8_t config_reg_value;
        BusI2c::read(i2c_address, REG_CONFIG, &read_config_reg_value, 1);

        // Update the config register value
        config_reg_value = (config_reg_value & 0x0F) | enabled_channels << 4;

        BusI2c::write(i2c_address, REG_CONFIG, &config_reg_value, 1);

        pointToConversionResult();
    }

    void disableChannel(Channel channel)
    {
        uint8_t channel = static_cast<uint8_t> channel;
        enabled_channels &= ~(1 << channel);

        uint8_t config_reg_value;
        BusI2c::read(i2c_address, REG_CONFIG, &read_config_reg_value, 1);

        // Update the config register value
        config_reg_value = (config_reg_value & 0x0F) | enabled_channels << 4;

        BusI2c::write(i2c_address, REG_CONFIG, &config_reg_value, 1);

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
        // Trigger a conversion
        CONVST::high();
        miosix::delayUs(1);
        CONVST::low();

        // TODO: Do we have to wait longer if multiple channels are enabled?
        // Wait for the conversion to complete
        miosix::delayUs(2);

        uint8_t data[2];
        BusI2c::read(i2c_address, REG_CONVERSION_RESULT, &data, 2);
        uint16_t val = data[0] << 8 | data[1];

        return true;
    }

    bool selfTest() { return true; }

private:
    // Address of the AD7994 on the I2C bus
    const uint8_t i2c_address;

    // The 4 LSBs indicate where the corresponding channel is enabled or not.
    uint8_t enabled_channels;

    /**
     * @brief Writes the conversion register address to the address pointer
     * register in order to restore the class invariant.
     */
    void pointToConversionResult()
    {
        BusI2c::directWrite(i2c_address, &REG_CONVERSION_RESULT, 1);
    }

    enum Registers : uint8_t
    {
        REG_CONVERSION_RESULT = 0b0000,
        REG_ALERT_STATUS      = 0b0001,
        REG_CONFIG            = 0b0010,

    };
};

#endif /* SRC_SHARED_DRIVERS_ADC_AD7994_H */
