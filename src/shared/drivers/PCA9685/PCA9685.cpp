/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Riccardo Sironi
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

#include "PCA9685.h"

#include <drivers/timer/TimestampTimer.h>

namespace Boardcore
{

PCA9685::PCA9685(I2C& i2c, I2CDriver::I2CSlaveConfig i2cConfig,
                 uint8_t prescale, PCA9685::OutputType outputType,
                 bool inverted, bool externalClock)
    : i2c(i2c), i2cConfig(i2cConfig), prescale(prescale),
      outputType(outputType), inverted(inverted), externalClock(externalClock)
{
}

bool PCA9685::init()
{
    if (isInitialized)
    {
        lastError = ALREADY_INIT;
        return false;
    }

    // Trying to probe the sensor to check if it is connected
    if (!i2c.probe(i2cConfig))
    {
        lastError = BUS_FAULT;
        return false;
    }

    uint8_t mode1;
    // read current MODE1 register value
    if (!i2c.readRegister(i2cConfig, Register::MODE1, mode1))
    {
        LOG_ERR(logger, "Failed to read MODE1 register");
        lastError = BUS_FAULT;
        return false;
    }
    mode1 |= Mode1BitMask::SLEEP | Mode1BitMask::ALLCALL;
    // Set Sleep and ALLCALL bits
    if (!i2c.writeRegister(i2cConfig, Register::MODE1, mode1))
    {
        LOG_ERR(logger, "Failed to write MODE1 register");
        lastError = BUS_FAULT;
        return false;
    }

    // Wait for oscillator to stabilize
    miosix::Thread::sleep(1);

    // Set prescale value
    if (!i2c.writeRegister(i2cConfig, Register::PRE_SCALE, prescale))
    {
        LOG_ERR(logger, "Failed to write PRE_SCALE register");
        lastError = BUS_FAULT;
        return false;
    }

    if (externalClock)
        // set EXTCLK bit, if using an external clock we don't need to clear
        mode1 |= Mode1BitMask::EXTCLK;
    else
        // clear Sleep bit to start the internal oscillator
        mode1 &= ~Mode1BitMask::SLEEP;

    if (!i2c.writeRegister(i2cConfig, Register::MODE1, mode1))
    {
        LOG_ERR(logger, "Failed to write MODE1 register");
        lastError = BUS_FAULT;
        return false;
    }

    // Wait for oscillator to stabilize
    miosix::Thread::sleep(1);

    isInitialized = true;

    return true;
}

bool PCA9685::setAllCall(bool enable)
{
    uint8_t mode1;
    // read current MODE1 register value
    if (!i2c.readRegister(i2cConfig, Register::MODE1, mode1))
    {
        LOG_ERR(logger, "Failed to read MODE1 register");
        lastError = BUS_FAULT;
        return false;
    }
    if (enable)
        mode1 |= Mode1BitMask::ALLCALL;  // Set ALLCALL bit
    else
        mode1 &= ~Mode1BitMask::ALLCALL;  // Clear ALLCALL bit

    // write updated MODE1 register value
    if (!i2c.writeRegister(i2cConfig, Register::MODE1, mode1))
    {
        LOG_ERR(logger, "Failed to write MODE1 register");
        lastError = BUS_FAULT;
        return false;
    }

    return true;
}

bool PCA9685::setPWMFrequency(uint8_t prescale)
{
    uint8_t mode1;
    // read current MODE1 register value
    if (!i2c.readRegister(i2cConfig, Register::MODE1, mode1))
    {
        LOG_ERR(logger, "Failed to read MODE1 register");
        lastError = BUS_FAULT;
        return false;
    }

    // set Sleep bit
    mode1 |= Mode1BitMask::SLEEP;
    if (!i2c.writeRegister(i2cConfig, Register::MODE1, mode1))
    {
        LOG_ERR(logger, "Failed to write MODE1 register");
        lastError = BUS_FAULT;
        return false;
    }
    // set prescale value
    if (!i2c.writeRegister(i2cConfig, Register::PRE_SCALE, prescale))
    {
        LOG_ERR(logger, "Failed to write PRE_SCALE register");
        lastError = BUS_FAULT;
        return false;
    }
    // clear Sleep bit
    mode1 &= ~Mode1BitMask::SLEEP;
    if (!i2c.writeRegister(i2cConfig, Register::MODE1, mode1))
    {
        LOG_ERR(logger, "Failed to write MODE1 register");
        lastError = BUS_FAULT;
        return false;
    }
    return true;
}

bool PCA9685::softwareReset()
{
    // The software reset is performed by issuing an I2C General Call (0x00)
    // with the SWRST command (0x06)
    I2CDriver::I2CSlaveConfig generalCall = i2cConfig;
    generalCall.slaveAddress              = 0x00;  // I2C General Call address

    const uint8_t swrst = 0x06;
    if (!i2c.write(generalCall, &swrst, 1))
    {
        LOG_ERR(logger, "Failed to send software reset command");
        lastError = BUS_FAULT;
        return false;
    }

    // Wait for the reset to complete
    // According to the datasheet, the oscillator needs 500us to stabilize
    // We wait 1ms just to be safe
    miosix::Thread::sleep(1);

    return true;
}

bool PCA9685::setOutputState(bool totem_pole, bool invert)
{
    uint8_t mode2;
    if (!i2c.readRegister(i2cConfig, Register::MODE2, mode2))
    {
        LOG_ERR(logger, "Failed to read MODE2 register");
        lastError = BUS_FAULT;
        return false;
    }
    if (totem_pole)
        mode2 |= Mode2BitMask::TOTEM_POLE;  // Set totem pole
    else
        mode2 &= ~Mode2BitMask::TOTEM_POLE;  // Set open drain

    if (invert)
        mode2 |= Mode2BitMask::INVERT;  // Set invert bit
    else
        mode2 &= ~Mode2BitMask::INVERT;  // Clear invert bit

    if (!i2c.writeRegister(i2cConfig, Register::MODE2, mode2))
    {
        LOG_ERR(logger, "Failed to write MODE2 register");
        lastError = BUS_FAULT;
        return false;
    }

    return true;
}

bool PCA9685::enableAutoIncrement(bool enable)
{
    uint8_t mode1;
    if (!i2c.readRegister(i2cConfig, Register::MODE1, mode1))
    {
        LOG_ERR(logger, "Failed to read MODE1 register");
        lastError = BUS_FAULT;
        return false;
    }
    if (enable)
        mode1 |= Mode1BitMask::AI;
    else
        mode1 &= ~Mode1BitMask::AI;

    if (!i2c.writeRegister(i2cConfig, Register::MODE1, mode1))
    {
        LOG_ERR(logger, "Failed to write MODE1 register");
        lastError = BUS_FAULT;
        return false;
    }
    return true;
}

bool PCA9685::setPWM(PCA9685Utils::Channel channel, uint16_t on, uint16_t off)
{
    if (on == off && on != 0)
    {
        LOG_ERR(logger,
                "Invalid PWM values: on and off cannot be the same value");
        lastError = COMMAND_FAILED;
        return false;
    }  // The LEDn_ON and LEDn_OFF count registers should never be programmed
       // with the same values.

    uint8_t led_n_on_l =
        Register::CHANNEL_BASE + 4 * static_cast<uint8_t>(channel);
    uint8_t led_n_on_h  = led_n_on_l + 1;
    uint8_t led_n_off_l = led_n_on_l + 2;
    uint8_t led_n_off_h = led_n_on_l + 3;

    // Since we're working with 12-bit values, mask the upper bits

    // Mask the lower 8 bits
    uint8_t on_8_lsb = on & 0xFF;
    // Mask the upper 4 bits by shifting right 8 bits and masking with 0x0F
    uint8_t on_3_msb = (on >> 8) & 0x0F;

    uint8_t off_8_lsb = off & 0xFF;
    uint8_t off_3_msb = (off >> 8) & 0x0F;

    // Write to ALL_LED registers
    if (!i2c.writeRegister(i2cConfig, led_n_on_l, on_8_lsb) ||
        !i2c.writeRegister(i2cConfig, led_n_on_h, on_3_msb) ||
        !i2c.writeRegister(i2cConfig, led_n_off_l, off_8_lsb) ||
        !i2c.writeRegister(i2cConfig, led_n_off_h, off_3_msb))
    {
        LOG_ERR(logger, "Failed to write LED registers for channel {}",
                static_cast<uint8_t>(channel));
        lastError = BUS_FAULT;
        return false;
    }
    return true;
}

bool PCA9685::setDutyCycle(PCA9685Utils::Channel channel, float dutyCycle)
{
    if (dutyCycle < 0.0f)
        dutyCycle = 0.0f;
    else if (dutyCycle > 1.0f)
        dutyCycle = 1.0f;

    uint16_t off = static_cast<uint16_t>(dutyCycle * 4095.0f);
    return setPWM(channel, 0, off);
}

bool PCA9685::setAllPWM(uint16_t on, uint16_t off)
{
    if (on > 4095 || off > 4095 || (on == off && on != 0))
    {
        LOG_ERR(logger,
                "Invalid PWM values: on and off cannot be the same value");
        lastError = COMMAND_FAILED;
        return false;
    }  // Invalid on/off values
    // The LEDn_ON and LEDn_OFF count registers should never be programmed
    // with the same values.
    // Since we're working with 12-bit values, mask the upper bits

    // Mask the lower 8 bits
    uint8_t on_8_lsb = on & 0xFF;
    // Mask the upper 4 bits by shifting right 8 bits and masking with 0x0F
    uint8_t on_3_msb  = (on >> 8) & 0x0F;
    uint8_t off_8_lsb = off & 0xFF;
    uint8_t off_3_msb = (off >> 8) & 0x0F;

    // Write to ALL_LED registers
    if (!i2c.writeRegister(i2cConfig, Register::ALL_LED_ON_L, on_8_lsb) ||
        !i2c.writeRegister(i2cConfig, Register::ALL_LED_ON_H, on_3_msb) ||
        !i2c.writeRegister(i2cConfig, Register::ALL_LED_OFF_L, off_8_lsb) ||
        !i2c.writeRegister(i2cConfig, Register::ALL_LED_OFF_H, off_3_msb))
    {
        LOG_ERR(logger, "Failed to write ALL_LED registers");
        lastError = BUS_FAULT;
        return false;
    }
    return true;
}
bool PCA9685::setAllDutyCycle(float dutyCycle)
{
    if (dutyCycle < 0.0f)
        dutyCycle = 0.0f;
    else if (dutyCycle > 1.0f)
        dutyCycle = 1.0f;
    uint16_t off = static_cast<uint16_t>((dutyCycle * 4095.0f));
    return setAllPWM(0, off);
}

int PCA9685::prescaleCalculation(float targetFreq, float clkFreq)
{
    // prevent using the wrong clock frequency if not set
    if (!externalClock)
        clkFreq = 25000000.0f;

    // According to the datasheet, the prescale value is computed as:
    // prescale = round(clock / (4096 * target_frequency)) - 1
    float prescaleVal =
        static_cast<float>(clkFreq) / (4096.0f * targetFreq) - 1.0f;
    // Round to the nearest integer
    return static_cast<int>(prescaleVal + 0.5f);
}

Boardcore::SensorErrors PCA9685::getLastError() { return lastError; }

}  // namespace Boardcore

