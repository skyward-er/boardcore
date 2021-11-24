/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <drivers/timer/GeneralPurposeTimer.h>
#include <drivers/timer/TimerUtils.h>
#include <interfaces/arch_registers.h>

#include "SPIBusInterface.h"

/**
 * @brief Generates SPI clock and chip select signal through two timers chained
 * together.
 *
 * GPIO must be already configured.
 */
template <int ChainChannel = 2, int CSChanel = 2, int SCKChannel = 4>
class SPISignalGenerator
{
    static_assert(ChainChannel >= 1 && ChainChannel <= 4,
                  "ChainChannel must be betwen 1 and 4");
    static_assert(CSChanel >= 1 && CSChanel <= 4,
                  "CSChanel must be betwen 1 and 4");
    static_assert(SCKChannel >= 1 && SCKChannel <= 4,
                  "SCKChannel must be betwen 1 and 4");

public:
    /**
     * @brief Create a SPISignalGenerator object. This does not configure the
     * timers.
     *
     * @param nBytes Number of bytes for the sck signal.
     * @param transactionFrequency Frequency of the SPI transactions.
     * @param spiFrequency SPI clock signal frequency.
     * @param spiMode SPI mode, this affects the clock polarity and phase.
     * @param masterTimer Master timer which generates the CS signal.
     * @param slaveTimer Slave timer which generates the SCK signal.
     * @param slaveTriggerSource Trigger source form the slave timer. This
     * depends on the timers combination.
     */
    SPISignalGenerator(
        int nBytes, int transactionFrequency, int spiFrequency = 1e6,
        SPIMode spiMode = SPIMode::MODE0, TIM_TypeDef *masterTimer = TIM1,
        TIM_TypeDef *slaveTimer = TIM3,
        GeneralPurposeTimer<uint16_t>::TriggerSource slaveTriggerSource =
            GeneralPurposeTimer<uint16_t>::TriggerSource::ITR0);

    /**
     * @brief Sets up the two timers.
     */
    void configure();

    /**
     * @brief Enables SPI signal generation.
     */
    void enable();

    /**
     * @brief Disables SPI signal generation.
     */
    void disable();

private:
    int nBytes;                ///< SPI Clock pulses.
    int transactionFrequency;  ///< Frequency of the transactions are generated.
    int spiFrequency;          ///< SPI Clock frequency.
    SPIMode spiMode;
    GeneralPurposeTimer<uint16_t>
        masterTimer;  ///< Master timer for CS generation.
    GeneralPurposeTimer<uint16_t>
        slaveTimer;  ///< Slave timer for SCK generation.
    GeneralPurposeTimer<uint16_t>::TriggerSource slaveTriggerSource;
};

template <int ChainChannel, int CSChanel, int SCKChannel>
SPISignalGenerator<ChainChannel, CSChanel, SCKChannel>::SPISignalGenerator(
    int nBytes, int transactionFrequency, int spiFrequency, SPIMode spiMode,
    TIM_TypeDef *masterTimer, TIM_TypeDef *slaveTimer,
    GeneralPurposeTimer<uint16_t>::TriggerSource slaveTriggerSource)
    : nBytes(nBytes), transactionFrequency(transactionFrequency),
      spiFrequency(spiFrequency), spiMode(spiMode), masterTimer(masterTimer),
      slaveTimer(slaveTimer), slaveTriggerSource(slaveTriggerSource)
{
}

template <int ChainChannel, int CSChanel, int SCKChannel>
void SPISignalGenerator<ChainChannel, CSChanel, SCKChannel>::configure()
{
    // Configure master timer
    {
        masterTimer.reset();

        // Connect the specified channel to the trigger output
        switch (ChainChannel)
        {
            case 1:
                masterTimer.setMasterMode(
                    GeneralPurposeTimer<uint16_t>::MasterMode::OC1REF_OUTPUT);
                break;
            case 2:
                masterTimer.setMasterMode(
                    GeneralPurposeTimer<uint16_t>::MasterMode::OC2REF_OUTPUT);
                break;
            case 3:
                masterTimer.setMasterMode(
                    GeneralPurposeTimer<uint16_t>::MasterMode::OC3REF_OUTPUT);
                break;
            case 4:
                masterTimer.setMasterMode(
                    GeneralPurposeTimer<uint16_t>::MasterMode::OC4REF_OUTPUT);
                break;
        }

        // Set the prescaler and auto realod value
        masterTimer.setPrescaler(TimerUtils::computePrescalerValue(
            masterTimer.getTimer(), spiFrequency * 4));
        masterTimer.setAutoReloadRegister(spiFrequency * 4 /
                                          transactionFrequency);

        // Set channels capture/compare register
        uint16_t ccRegister = nBytes * 8;
        if (spiMode >= SPIMode::MODE2)
        {
            ccRegister += 1;
        }
        ccRegister *= 4;

        // Set chain channel
        masterTimer.setCaptureCompareRegister<ChainChannel>(ccRegister);
        masterTimer.setOutputCompareMode<ChainChannel>(
            GeneralPurposeTimer<uint16_t>::OutputCompareMode::PWM_MODE_1);

        // Set CS channel if different
        if (ChainChannel != CSChanel)
        {
            masterTimer.setCaptureCompareRegister<CSChanel>(ccRegister);
            masterTimer.setOutputCompareMode<CSChanel>(
                GeneralPurposeTimer<uint16_t>::OutputCompareMode::PWM_MODE_2);

            // Enable CS capture/compare output
            masterTimer.enableCaptureCompareOutput<CSChanel>();
        }
        else
        {
            // If chain channel anch cs channel are the same, the cs output
            // must be from the complementary output
            masterTimer.enableCaptureCompareComplementaryOutput<ChainChannel>();
            masterTimer.setCaptureCompareComplementaryPolarity<ChainChannel>(
                GeneralPurposeTimer<
                    uint16_t>::OutputComparePolarity::ACTIVE_LOW);
        }

        // Update the registers
        masterTimer.generateUpdate();
    }

    // Configure slave timer
    {
        slaveTimer.reset();

        // Set slaveTimer in gated mode
        slaveTimer.setSlaveMode(
            GeneralPurposeTimer<uint16_t>::SlaveMode::GATED_MODE);

        // Set ITR1 as internal trigger source
        slaveTimer.setTriggerSource(slaveTriggerSource);

        // Set the prescaler and auto realod value
        slaveTimer.setPrescaler(TimerUtils::computePrescalerValue(
            slaveTimer.getTimer(), spiFrequency * 4));
        slaveTimer.setAutoReloadRegister(1);

        // Set SCK capture/compare register
        slaveTimer.setCaptureCompareRegister<SCKChannel>(1);

        // Set channel 1 to toggle mode
        slaveTimer.setOutputCompareMode<SCKChannel>(
            GeneralPurposeTimer<uint16_t>::OutputCompareMode::TOGGLE);
        if (spiMode >= SPIMode::MODE2)
        {
            slaveTimer.setCaptureComparePolarity<SCKChannel>(
                GeneralPurposeTimer<
                    uint16_t>::OutputComparePolarity::ACTIVE_LOW);
        }

        // Enable capture/compare output
        slaveTimer.enableCaptureCompareOutput<SCKChannel>();

        // Update the register
        slaveTimer.generateUpdate();
    }
}

template <int ChainChannel, int CSChanel, int SCKChannel>
void SPISignalGenerator<ChainChannel, CSChanel, SCKChannel>::enable()
{
    slaveTimer.enable();
    masterTimer.enable();
}

template <int ChainChannel, int CSChanel, int SCKChannel>
void SPISignalGenerator<ChainChannel, CSChanel, SCKChannel>::disable()
{
    masterTimer.disable();
    slaveTimer.disable();
}