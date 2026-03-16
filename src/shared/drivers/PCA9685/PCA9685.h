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

#pragma once

#include <drivers/i2c/I2C.h>
#include <sensors/Sensor.h>

#include "PCA9685Utils.h"

namespace Boardcore
{
class PCA9685
{
public:
    enum class OutputType : uint8_t
    {
        OPEN_DRAIN,
        TOTEM_POLE
    };

    /**
     * @brief PCA9685 Constructor.
     * @param i2c bus on which the sensor lays.
     * @param i2cConfig I2C configuration struct for the sensor.
     * @param prescale Prescale value for setting the PWM frequency. - Defaults
     * to 0x11 (339 Hz).
     * @param outputType Output driver type (open-drain or totem-pole).
     * @param inverted true to invert the output logic, false for normal logic.
     * @return true if setting the mode succeeded, false otherwise.
     */
    PCA9685(I2C& i2c, I2CDriver::I2CSlaveConfig i2cConfig,
            uint8_t prescale               = 0x12,
            PCA9685::OutputType outputType = OutputType::OPEN_DRAIN,
            bool inverted = true, bool externalClock = false);
    ~PCA9685() {};

    /**
     * @brief Initializes the board with the current settings.
     * @return true if initialization succeeded, false otherwise.
     */
    bool init();
    /**
     * @brief When the ALLCALL bit is set the device only listens to the ALL
     * CALL I2C-bus address (0xE0 by default).
     * @return true if setting the bit succeeded, false otherwise.
     */
    bool setAllCall(bool enable);
    /**
     * @brief Sets the PWM frequency by writing the prescale to the register.
     * @return true if writing to the register succeeded, false otherwise.
     */
    bool setPWMFrequency(uint8_t prescale);
    /**
     * @brief Issues a software reset to all PCA9685 on the bus.
     * @return true if the reset command was sent successfully, false otherwise.
     */
    bool softwareReset();
    /**
     * @brief Sets the output driver mode.
     * @param totem_pole true for totem-pole structure, false for open-drain.
     * @param invert true to invert the output logic, false for normal logic.
     * @return true if setting the mode succeeded, false otherwise.
     */
    bool setOutputState(bool totem_pole, bool invert = false);
    /**
     * @brief Enables or disables the auto-increment bit.
     * @param enable true to enable auto-increment, false to disable.
     * @return true if setting the bit succeeded, false otherwise.
     */
    bool enableAutoIncrement(bool enable);

    /**
     * @brief Sets the duty cycle on a specific channel
     * @param channel Channel number (0-15)
     * @param dutyCycle Duty cycle percentage (0.0 - 1.0)
     * @return true if setting the duty cycle succeeded, false otherwise.
     */
    bool setDutyCycle(PCA9685Utils::Channel channel, float dutyCycle);
    /**
     * @brief Sets the duty cycle on all channels
     * @param dutyCycle Duty cycle percentage (0.0 - 1.0)
     * @return true if setting the duty cycle succeeded, false otherwise.
     */
    bool setAllDutyCycle(float dutyCycle);

    /**
     * @return The last error encountered by the driver.
     */
    SensorErrors getLastError();

    /**
     * @brief Sets the 12-bit PWM signal on a specific channel
     * @param channel Channel number (0-15)
     * @param on Tick where the signal should turn ON (0-4095)
     * @param off Tick where the signal should turn OFF (0-4095)
     *
     * Note: on and off cannot be equal.
     * @return true if setting the bits succeeded, false otherwise.
     */
    bool setPWM(PCA9685Utils::Channel channel, uint16_t on, uint16_t off);
    /**
     * @brief Sets the 12-bit PWM signal on all channels
     * @param on Tick where the signal should turn ON (0-4095)
     * @param off Tick where the signal should turn OFF (0-4095)
     * @return true if setting the duty cycle succeeded, false otherwise.
     */
    bool setAllPWM(uint16_t on, uint16_t off);

    /**
     * @brief Sets the 12-bit PWM signal on all channels
     * @param targetFreq Target Frequency in Hz. BEWARE: the prescale value can
     * only be an integer. Thus, the real frequency can (and probably will)
     * differ from the target frequency.
     * @param clkFreq Clock Frequency in Hz. Default is 25 MHz.
     * @return true if setting the duty cycle succeeded, false otherwise.
     */
    int prescaleCalculation(float targetFreq, float clkFreq = 25000000.0f);

private:
    enum Register : uint8_t
    {
        MODE1         = 0x00,
        MODE2         = 0x01,
        PRE_SCALE     = 0xFE,
        ALLCALLADR    = 0xE0,
        CHANNEL_BASE  = 0x06,  // Base address for channel 0
        ALL_LED_ON_L  = 0xFA,
        ALL_LED_ON_H  = 0xFB,
        ALL_LED_OFF_L = 0xFC,
        ALL_LED_OFF_H = 0xFD
    };

    enum Mode1BitMask : uint8_t
    {
        RESTART = 0x80,  // Not used in this driver
        EXTCLK  = 0x40,
        AI      = 0x20,
        SLEEP   = 0x10,
        ALLCALL = 0x01
    };

    enum Mode2BitMask : uint8_t
    {
        TOTEM_POLE = 0x04,
        INVERT     = 0x10
    };

    // I2C address and speed mode
    // I2C bus on which the sensor lays
    I2C& i2c;

    I2CDriver::I2CSlaveConfig i2cConfig;
    /* Prescale value for setting the PWM frequency, computed as:
     *
     * prescale = round(Clock / (4096 * target-frequency)) - 1
     *
     * The internal clock is 25Mhz
     */
    uint8_t prescale;

    // Output driver type (open-drain or totem-pole)
    PCA9685::OutputType outputType;

    // invert the output logic
    uint8_t inverted;

    bool externalClock = false;

    bool isInitialized = false;

    PrintLogger logger = Logging::getLogger("pca9685");

protected:
    SensorErrors lastError = SensorErrors::NO_ERRORS;
};
}  // namespace Boardcore
