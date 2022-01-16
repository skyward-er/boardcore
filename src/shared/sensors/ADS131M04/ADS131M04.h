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

#include <diagnostic/PrintLogger.h>
#include <drivers/spi/SPIDriver.h>
#include <sensors/Sensor.h>

#include "ADS131M04Data.h"

namespace Boardcore
{

/**
 * @brief Driver for ADS131M04 4 simultaneous channels adc.
 *
 * The ADS131M04 is a four-channel, simultaneously-sampling, 24-bit,
 * delta-sigma (ΔΣ), analog-to-digital converter (ADC). The individual ADC
 * channels can be independently configured depending on the sensor input. A
 * low-noise, programmable gain amplifier (PGA) provides gains ranging from 1
 * to 128 to amplify low-level signals.
 *
 * Each channel on the ADS131M04 contains a digital decimation filter that
 * demodulates the output of the ΔΣ modulators. Offset and gain calibration
 * registers can be programmed to automatically adjust output samples for
 * measured offset and gain errors.
 *
 * The device features a "global-chop mode" to reduce offset error and offset
 * drift inherent to the device due to mismatch in the internal circuitry to
 * very low levels. When global-chop mode is enabled by setting the GC_EN bit in
 * the GLOBAL_CHOP_CFG register, the device uses the conversion results from two
 * consecutive internal conversions taken with opposite input polarity to cancel
 * the device offset voltage.
 *
 * Each channel has a dedicated input multiplexer that controls which signals
 * are routed to the ADC channels:
 * - The analog input pins corresponding to the given channel
 * - AGND, which is helpfu for offset calibraton
 * - Positive DC test signal
 * - Negative DC test signal
 *
 * Each channel also feature an integrated programmable gain amplifier (PGA)
 * that provides gains of 1 to 128. Each channel has an independent PGA.
 *
 * The device communicates via SPI, the maximum allowed frequency is 25MHz.
 *
 * The ADC can work on 3 different power configurations depending on the clock
 * frequency:
 * - High-resulution: 250-32KSPS
 * - Low-power: 125-16KSPS
 * - Very-low-power: 62.5-8KSPS
 * You will probably have the ADC in high resolution mode.
 */
class ADS131M04 : public Sensor<ADS131M04Data>
{
public:
    /**
     * @brief ADC's oversampling ratio configurations.
     *
     * The OSR determins the output data rate, depending on the master clock
     * frequency.
     *
     * ODR = f_CLK / 2 / OSR
     *
     * On Skyward's boards an 8.192MHz clock is used.
     */
    enum class OversamplingRatio : uint16_t
    {
        OSR_128   = 0,         // ODR is 32KHz
        OSR_256   = 0x1 << 2,  // ODR is 16KHz
        OSR_512   = 0x2 << 2,  // ODR is 8KHz
        OSR_1024  = 0x3 << 2,  // ODR is 4KHz
        OSR_2048  = 0x4 << 2,  // ODR is 2KHz
        OSR_4096  = 0x5 << 2,  // ODR is 1KHz
        OSR_8192  = 0x6 << 2,  // ODR is 500Hz
        OSR_16256 = 0x7 << 2   // ODR is 250Hz
    };

    enum class PGA : uint16_t
    {
        PGA_1   = 0,    ///< Full scale resolution is ±1.2V
        PGA_2   = 0x1,  ///< Full scale resolution is ±600mV
        PGA_4   = 0x2,  ///< Full scale resolution is ±300mV
        PGA_8   = 0x3,  ///< Full scale resolution is ±150mV
        PGA_16  = 0x4,  ///< Full scale resolution is ±75mV
        PGA_32  = 0x5,  ///< Full scale resolution is ±37.5mV
        PGA_64  = 0x6,  ///< Full scale resolution is ±18.75mV
        PGA_128 = 0x7   ///< Full scale resolution is ±9.375mV
    };

    enum class Channel : int
    {
        CHANNEL_0 = 0,
        CHANNEL_1 = 1,
        CHANNEL_2 = 2,
        CHANNEL_3 = 3
    };

    ADS131M04(SPIBusInterface &bus, miosix::GpioPin cs,
              SPIBusConfig config = getDefaultSPIConfig());

    explicit ADS131M04(SPISlave spiSlave);

    /**
     * Constructs the default config for SPI Bus.
     *
     * @returns The default SPIBusConfig.
     */
    static SPIBusConfig getDefaultSPIConfig();

    bool init() override;

    bool reset();

    /**
     * @brief Samples each channel, averages the samples and applies sets up
     * offset compensation in the device.
     */
    void calibrateOffset();

    void setOversamplingRatio(OversamplingRatio ratio);

    void setChannelPGA(Channel channel, PGA gain);

    /**
     * @brief Sets the channel offset.
     *
     * Note that the device offset is a 24bit two complement.
     */
    void setChannelOffset(Channel channel, uint32_t offset);

    /**
     * @brief Sets the channel gain calibration.
     *
     * The ADS131M04 corrects for gain errors by multiplying the ADC conversion
     * result using the gain calibration registers.
     * The gain calibration value is interpreted as a 24bit unsigned. The values
     * corresponds to n * (1/2^23), rainging from 0 to 2 - (1/2^23).
     *
     * This function accepts a value between 0 and 2, it then compute the
     * correct gain register value.
     *
     * @param gain Must be between 0 and 2.
     */
    void setChannelGainCalibration(Channel channel, double gain);

    void enableChannel(Channel channel);

    void disableChannel(Channel channel);

    void enableGlobalChopMode();

    void disableGlobalChopMode();

    bool selfTest() override;

protected:
    ADS131M04Data sampleImpl() override;

private:
    enum class Registers : uint16_t
    {
        // Device settings and indicators
        REG_ID     = 0,
        REG_STATUS = 0x1,

        // Global settings across channels
        REG_MODE        = 0x2,
        REG_CLOCK       = 0x3,
        REG_GAIN        = 0x4,
        REG_CFG         = 0x6,
        REG_THRSHLD_MSB = 0x7,
        REG_THRSHLD_LSB = 0x8,

        // Channel specific settings
        REG_CH0_CFG      = 0x9,
        REG_CH0_OCAL_MSB = 0xA,
        REG_CH0_OCAL_LSB = 0xB,
        REG_CH0_GCAL_MSB = 0xC,
        REG_CH0_GCAL_LSB = 0xD,
        REG_CH1_CFG      = 0xE,
        REG_CH1_OCAL_MSB = 0xF,
        REG_CH1_OCAL_LSB = 0x10,
        REG_CH1_GCAL_MSB = 0x11,
        REG_CH1_GCAL_LSB = 0x12,
        REG_CH2_CFG      = 0x13,
        REG_CH2_OCAL_MSB = 0x14,
        REG_CH2_OCAL_LSB = 0x15,
        REG_CH2_GCAL_MSB = 0x16,
        REG_CH2_GCAL_LSB = 0x17,
        REG_CH3_CFG      = 0x18,
        REG_CH3_OCAL_MSB = 0x19,
        REG_CH3_OCAL_LSB = 0x1A,
        REG_CH3_GCAL_MSB = 0x1B,
        REG_CH3_GCAL_LSB = 0x1C,

        // Register map CRC
        REG_REGMAP_CRC = 0x3E
    };

    uint16_t readRegister(Registers reg);

    void writeRegister(Registers reg, uint16_t data);

    void changeRegister(Registers reg, uint16_t newValue, uint16_t mask);

    enum class Commands : uint16_t
    {
        NULL_CMD = 0,
        RESET    = 0x11,
        STANDBY  = 0x22,
        WAKEUP   = 0x33,
        LOCK     = 0x555,
        UNLOCK   = 0x655,
        RREG     = 0xA000,
        WREG     = 0x6000
    };

    ///< Digit value in mV for each pga configurations
    const float PGA_LSB_SIZE[8] = {143.0511e-9, 71.5256e-9, 35.7628e-9,
                                   17.8814e-9,  8.9407e-9,  4.4703e-9,
                                   2.2352e-9,   1.1176e-9};

    PGA channelsPGAGain[4] = {PGA::PGA_1};

protected:
    SPISlave spiSlave;

    PrintLogger logger = Logging::getLogger("ads131m04");
};

namespace ADS131M04RegisterBitMasks
{

// Status register
constexpr uint16_t REG_STATUS_LOCK     = 1 << 15;
constexpr uint16_t REG_STATUS_F_RESYNC = 1 << 14;
constexpr uint16_t REG_STATUS_REG_MAP  = 1 << 13;
constexpr uint16_t REG_STATUS_CRC_ERR  = 1 << 12;
constexpr uint16_t REG_STATUS_CRC_TYPE = 1 << 11;
constexpr uint16_t REG_STATUS_RESET    = 1 << 10;
constexpr uint16_t REG_STATUS_WLENGTH  = 3 << 8;
constexpr uint16_t REG_STATUS_DRDY3    = 1 << 3;
constexpr uint16_t REG_STATUS_DRDY2    = 1 << 2;
constexpr uint16_t REG_STATUS_DRDY1    = 1 << 1;
constexpr uint16_t REG_STATUS_DRDY0    = 1;

// Mode register
constexpr uint16_t REG_MODE_REG_CRC_EN = 1 << 13;
constexpr uint16_t REG_MODE_RX_CRC_EN  = 1 << 12;
constexpr uint16_t REG_MODE_CRC_TYPE   = 1 << 11;
constexpr uint16_t REG_MODE_RESET      = 1 << 10;
constexpr uint16_t REG_MODE_WLENGTH    = 3 << 8;
constexpr uint16_t REG_MODE_TIMEOUT    = 1 << 4;
constexpr uint16_t REG_MODE_DRDY_SEL   = 3 << 2;
constexpr uint16_t REG_MODE_DRDY_HiZ   = 1 << 1;
constexpr uint16_t REG_MODE_DRDY_FMT   = 1 << 0;

// Clock register
constexpr uint16_t REG_CLOCK_CH3_EN = 1 << 11;
constexpr uint16_t REG_CLOCK_CH2_EN = 1 << 10;
constexpr uint16_t REG_CLOCK_CH1_EN = 1 << 9;
constexpr uint16_t REG_CLOCK_CH0_EN = 1 << 8;
constexpr uint16_t REG_CLOCK_OSR    = 7 << 2;
constexpr uint16_t REG_CLOCK_PWR    = 3;

// Gain register
constexpr uint16_t REG_GAIN_PGAGAIN3 = 7 << 12;
constexpr uint16_t REG_GAIN_PGAGAIN2 = 7 << 8;
constexpr uint16_t REG_GAIN_PGAGAIN1 = 7 << 4;
constexpr uint16_t REG_GAIN_PGAGAIN0 = 7;

// Configuration register
constexpr uint16_t REG_CFG_GC_DLY   = 0xF << 9;
constexpr uint16_t REG_CFG_GC_EN    = 1 << 8;
constexpr uint16_t REG_CFG_CD_ALLCH = 1 << 7;
constexpr uint16_t REG_CFG_CD_NUM   = 7 << 4;
constexpr uint16_t REG_CFG_CD_LEN   = 7 << 1;
constexpr uint16_t REG_CFG_CD_EN    = 1;

// Channel configuration register
constexpr uint16_t REG_CHx_CFG_PHASE     = 0x3FF << 6;
constexpr uint16_t REG_CHx_CFG_DCBLK_DIS = 1 << 2;
constexpr uint16_t REG_CHx_CFG_MUX       = 3;

}  // namespace ADS131M04RegisterBitMasks

}  // namespace Boardcore
