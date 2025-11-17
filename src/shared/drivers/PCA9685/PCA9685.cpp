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
    if (!i2c.readRegister(i2cConfig, Register::MODE1, mode1))
    {
        lastError = BUS_FAULT;
        return false;
    }
    mode1 |= Mode1BitMask::SLEEP |
             Mode1BitMask::ALLCALL;  // Set Sleep and ALLCALL bits
    // Setting the prescale value. Returns false if something goes wrong
    if (!i2c.writeRegister(i2cConfig, Register::MODE1, mode1))
    {
        lastError = BUS_FAULT;
        return false;
    }  // Set Sleep and ALLCALL bits

    // Wait for oscillator to stabilize
    miosix::Thread::sleep(1);

    if (!i2c.writeRegister(i2cConfig, Register::PRE_SCALE, prescale))
    {
        lastError = BUS_FAULT;
        return false;
    }  // Set prescale value
    mode1 &= ~Mode1BitMask::SLEEP;  // Clear Sleep bit
    if (!i2c.writeRegister(i2cConfig, Register::MODE1, mode1))
    {
        lastError = BUS_FAULT;
        return false;
    }  // Clear Sleep bit to start the internal oscillator

    // Wait for oscillator to stabilize
    miosix::Thread::sleep(1);

    isInitialized = true;
    return true;
}

bool PCA9685::setAllCall(bool enable)
{
    uint8_t mode1;
    if (!i2c.readRegister(i2cConfig, Register::MODE1, mode1))
    {
        lastError = BUS_FAULT;
        return false;
    }
    if (enable)
        mode1 |= Mode1BitMask::ALLCALL;  // Set ALLCALL bit
    else
        mode1 &= ~Mode1BitMask::ALLCALL;  // Clear ALLCALL bit

    if (!i2c.writeRegister(i2cConfig, Register::MODE1, mode1))
    {
        lastError = BUS_FAULT;
        return false;
    }

    return true;
}

bool PCA9685::setPWMFrequency(uint8_t prescale)
{
    uint8_t mode1;
    if (!i2c.readRegister(i2cConfig, Register::MODE1, mode1))
    {
        lastError = BUS_FAULT;
        return false;
    }
    mode1 |= Mode1BitMask::SLEEP;  // Set Sleep bit
    if (!i2c.writeRegister(i2cConfig, Register::MODE1, mode1))
    {
        lastError = BUS_FAULT;
        return false;
    }  // Enter Sleep
    if (!i2c.writeRegister(i2cConfig, Register::PRE_SCALE, prescale))
    {
        lastError = BUS_FAULT;
        return false;
    }  // Set prescale value
    mode1 &= ~Mode1BitMask::SLEEP;  // Clear Sleep bit
    if (!i2c.writeRegister(i2cConfig, Register::MODE1, mode1))
    {
        lastError = BUS_FAULT;
        return false;
    }  // Exit from Sleep
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
        lastError = BUS_FAULT;
        return false;
    }  // Software reset

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
        lastError = BUS_FAULT;
        return false;
    }
    if (enable)
        mode1 |= Mode1BitMask::AI;
    else
        mode1 &= ~Mode1BitMask::AI;

    if (!i2c.writeRegister(i2cConfig, Register::MODE1, mode1))
    {
        lastError = BUS_FAULT;
        return false;
    }
    return true;
}

bool PCA9685::setPWM(uint8_t channel, uint16_t on, uint16_t off)
{
    if (channel > 15 || on > 4095 || off > 4095)
    {
        lastError = COMMAND_FAILED;
        return false;
    }  // Invalid channel and/or on/off values

    if (on == off)
    {
        lastError = COMMAND_FAILED;
        return false;
    }  // The LEDn_ON and LEDn_OFF count registers should never be programmed
       // with the same values.

    uint8_t ledn_on_l  = Register::CHANNEL_BASE + 4 * channel;
    uint8_t ledn_on_h  = ledn_on_l + 1;
    uint8_t ledn_off_l = ledn_on_l + 2;
    uint8_t ledn_off_h = ledn_on_l + 3;

    // Since we're working with 12-bit values, mask the upper bits
    uint8_t on_8_lsb  = on & 0xFF;
    uint8_t on_3_msb  = (on >> 8) & 0x0F;
    uint8_t off_8_lsb = off & 0xFF;
    uint8_t off_3_msb = (off >> 8) & 0x0F;

    if (!i2c.writeRegister(i2cConfig, ledn_on_l, on_8_lsb) &&
        !i2c.writeRegister(i2cConfig, ledn_on_h, on_3_msb) &&
        !i2c.writeRegister(i2cConfig, ledn_off_l, off_8_lsb) &&
        !i2c.writeRegister(i2cConfig, ledn_off_h, off_3_msb))
    {
        lastError = BUS_FAULT;
        return false;
    }
}

bool PCA9685::setDutyCycle(uint8_t channel, float dutyCycle)
{
    if (channel > 15)
    {
        lastError = COMMAND_FAILED;
        return false;
    }  // Invalid channels

    if (dutyCycle < 0.0f)
        dutyCycle = 0.0f;
    else if (dutyCycle > 100.0f)
        dutyCycle = 100.0f;

    uint16_t off = static_cast<uint16_t>((dutyCycle / 100.0f) * 4095.0f);
    return setPWM(channel, 0, off);
}

bool PCA9685::setServoAngle(uint8_t channel, float angle)
{
    // TODO
}
bool PCA9685::setAllPWM(uint16_t on, uint16_t off)
{
    if (on > 4095 || off > 4095 || on == off)
    {
        lastError = COMMAND_FAILED;
        return false;
    }  // Invalid on/off values
    // The LEDn_ON and LEDn_OFF count registers should never be programmed
    // with the same values.
    // Since we're working with 12-bit values, mask the upper bits
    uint8_t on_8_lsb  = on & 0xFF;
    uint8_t on_3_msb  = (on >> 8) & 0x0F;
    uint8_t off_8_lsb = off & 0xFF;
    uint8_t off_3_msb = (off >> 8) & 0x0F;

    if (!i2c.writeRegister(i2cConfig, Register::ALL_LED_ON_L, on_8_lsb) &&
        !i2c.writeRegister(i2cConfig, Register::ALL_LED_ON_H, on_3_msb) &&
        !i2c.writeRegister(i2cConfig, Register::ALL_LED_OFF_L, off_8_lsb) &&
        !i2c.writeRegister(i2cConfig, Register::ALL_LED_OFF_H, off_3_msb))
    {
        lastError = BUS_FAULT;
        return false;
    }
}
bool PCA9685::setAllDutyCycle(float dutyCycle)
{
    if (dutyCycle < 0.0f)
        dutyCycle = 0.0f;
    else if (dutyCycle > 100.0f)
        dutyCycle = 100.0f;

    uint16_t off = static_cast<uint16_t>((dutyCycle / 100.0f) * 4095.0f);
    return setAllPWM(0, off);
}
}  // namespace Boardcore

