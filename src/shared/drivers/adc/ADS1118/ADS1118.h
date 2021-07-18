/* Copyright (c) 2020 Skyward Experimental Rocketry
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

/**
 * @file ADS1118.h
 * @author Alberto Nidasio (alberto.nidasio@skywarder.eu)
 * @brief Driver for ADS1118 adc
 * @version 1.0
 * @date 2020-11-17
 *
 * The ADS1118 is a 16-bit delta-sigma analog-to-digital converter.
 * It measures the difference of two inputs configured by a MUX. The device
 * allows also to read each of the four pins in reference to GND. A temperature
 * sensor is also included.
 *
 * The communication uses the spi protocol and the maximum allowed frequency is
 * 4MHz. Configuration is applied by writing a 16bit value to the device.
 * Sampled voltages or temperature readings are obtained also by reading a 16bit
 * value while writing the configuration.
 *
 * Data rate between 8Hz and 860Hz can be programmed and an internal
 * programmable gain aplifier can be set with a sensitivity range from ±0.256V
 * to ±6.144V (note that the inputs must remain between VCC or GND).
 *
 * The data rate should be choosen as low as possible to allow the delta-sigma
 * adc to average the input voltage (this allows a less noisy reading).
 *
 * The device can work in two modes:
 * - CONTIN_CONV_MODE: Continuosly read the last configured channel, when you
 * make a read you'll obtain the lates reading
 * - SINGLE_SHOT_MODE: A single conversion is performed when the configuration
 * is written
 *
 * The ADS1118 is a simple device, it has a single data register where it can
 * store the reading, therefore it can sample a single input at a time. sample()
 * cylces through the enabled channels one at a time and writes it's
 * configuration while reading the value of the previous written one.
 *
 * As an example if you need to read 4 inputs at 50Hz you should set all the
 * data rates at 250Hz (50Hz x * 4 = 200Hz) and call sample() at a rate of
 * 200Hz.
 *
 * An example for how to use the driver can be found in the test code
 * (src/tests/drivers/test-ads1118.cpp)
 */

#pragma once

#include "ADS1118Data.h"
#include "drivers/spi/SPIDriver.h"
#include "sensors/Sensor.h"

/**
 * @brief Driver class for ADS1118 adc
 *
 * This class allows to mange the device configuration.
 */
class ADS1118 : public Sensor<ADS1118Data>
{
public:
    enum ADS1118Mux
    {
        MUX_AIN0_AIN1 = 0x0,  ///< AINp is AIN0 and AINn is AIN1 (default)
        MUX_AIN0_AIN3 = 0x1,  ///< AINp is AIN0 and AINn is AIN3
        MUX_AIN1_AIN3 = 0x2,  ///< AINp is AIN1 and AINn is AIN3
        MUX_AIN2_AIN3 = 0x3,  ///< AINp is AIN2 and AINn is AIN3
        MUX_AIN0_GND  = 0x4,  ///< AINp is AIN0 and AINn is GND
        MUX_AIN1_GND  = 0x5,  ///< AINp is AIN1 and AINn is GND
        MUX_AIN2_GND  = 0x6,  ///< AINp is AIN2 and AINn is GND
        MUX_AIN3_GND  = 0x7   ///< AINp is AIN3 and AINn is GND
    };                        ///< Multiplexer values

    enum ADS1118Pga
    {
        FSR_6_144 = 0x0,  ///< FSR is ±6.144 V
        FSR_4_096 = 0x1,  ///< FSR is ±4.096 V
        FSR_2_048 = 0x2,  ///< FSR is ±2.048 V (default)
        FSR_1_024 = 0x3,  ///< FSR is ±1.024 V
        FSR_0_512 = 0x4,  ///< FSR is ±0.512 V
        FSR_0_256 = 0x5   ///< FSR is ±0.256 V
    };                    ///< Programmable gain amplifier values

    enum ADS1118Mode
    {
        CONTIN_CONV_MODE = 0x0,  ///< Continuous-conversion mode
        SINGLE_SHOT_MODE = 0x1   ///< Power-down and single-shot mode (default)
    };                           ///< Conversione mode values

    enum ADS1118DataRate
    {
        DR_8   = 0x0,  ///< 8 SPS
        DR_16  = 0x1,  ///< 16 SPS
        DR_32  = 0x2,  ///< 32 SPS
        DR_64  = 0x3,  ///< 64 SPS
        DR_128 = 0x4,  ///< 128 SPS (default)
        DR_250 = 0x5,  ///< 250 SPS
        DR_475 = 0x6,  ///< 475 SPS
        DR_860 = 0x7   ///< 860 SPS
    };                 ///< Data rate configuration values

    enum ADS1118TempMode
    {
        ADC_MODE         = 0x0,  ///< ADC mode (default)
        TEMP_SENSOR_MODE = 0x1   ///< Temperature sensor mode
    };                           ///< Temeprature or ADC mode values

    enum ADS1118PullUp
    {
        PULL_UP_DIS = 0x0,  ///< Pullup resistor disabled on DOUT pin
        PULL_UP_EN  = 0x1   ///< Pullup resistor enabled on DOUT pin (default)
    };                      ///< Pull up enable or disable values

    union ADS1118Config
    {
        struct
        {
            ADS1118Mode mode : 1;  ///< Device operating mode
            ADS1118Pga pga : 3;  ///< Programmable gain amplifier configuration
            ADS1118Mux mux : 3;  ///< Input multiplexer configuration
            uint8_t singleShot : 1;        ///< Single-shot conversion start
            uint8_t reserved : 1;          ///< Reserved, doesn't matter
            uint8_t noOp : 2;              ///< No operation
            ADS1118PullUp pullUp : 1;      ///< Pullup enable
            ADS1118TempMode tempMode : 1;  ///< Temperature sensor mode
            ADS1118DataRate rate : 3;      ///< Data rate
        } bits;  ///< Includes all the configuration bits
        struct
        {
            uint8_t msb;  ///< Byte MSB
            uint8_t lsb;  ///< Byte LSB
        } byte;           ///< Includes the msb and lsb bytes
        uint16_t word;    ///< Representation in word (16-bits) format
    };                    ///< Structure of configuration word

    struct ADS1118InputConfig
    {
        ADS1118Mux mux       = MUX_AIN0_AIN1;  ///< Input's mux configuration
        ADS1118DataRate rate = DR_128;  ///< Input's data rate configuration
    };                                  ///< Driver's input config

    static constexpr uint8_t VALID_OPERATION =
        0x1;  ///< Indicates a valid configuration

    static const ADS1118Config
        ADS1118_DEFAULT_CONFIG;  ///< Default configuration

    static constexpr int8_t TEMP_CHANNEL = 8;  ///< Temperature channel number

    static constexpr int8_t NUM_OF_CHANNELS = 9;

    static constexpr int8_t INVALID_CHANNEL = -1;

    /**
     * @brief Construct a new ADS1118 object specifing spi bus, spi config and
     * cs pin as well as device configuration
     */
    ADS1118(SPIBusInterface &bus, GpioPin cs, ADS1118Config config_,
            SPIBusConfig spiConfig = getDefaultSPIConfig());

    /**
     * @brief Construct a new ADS1118 object
     *
     * @param spiSlave_ Spi slave configured with spi interface, spi config and
     * ss pin
     * @param config_ Device main configuration used as default while enabling
     * channels
     * @param busyWait_ Enable busy wait instead normal wait (uses `delayUs`
     * instead of `sleep`), only useful when sampling at close to the maximum
     * frequency!
     * @param tempDivider_ Specify how many onSimpleUpdate calls between each
     * temperature reading
     */
    ADS1118(SPISlave spiSlave_, ADS1118Config config_ = ADS1118_DEFAULT_CONFIG,
            bool busyWait_ = false, int16_t tempDivider_ = 100);

    /**
     * Constructs the default config for SPI Bus.
     *
     * @returns the default SPIBusConfig
     */
    static SPIBusConfig getDefaultSPIConfig();

    /**
     * @brief Initialize the configuration
     *
     * It resets all the channels thus you must call init() before enabling any
     * channel
     */
    bool init() override;

    /**
     * @brief Enables the sapling of a specific mux configuration with the main
     * configuration specified in the constructor
     *
     * @param mux Mux configuration to enable
     */
    void enableInput(ADS1118Mux mux);

    /**
     * @brief Enables the sapling of a specific mux configuration
     *
     * @param mux Mux configuration to enable
     * @param rate Data rate for this specific channel
     * @param pga Gain for this specific channel
     */
    void enableInput(ADS1118Mux mux, ADS1118DataRate rate, ADS1118Pga pga);

    /**
     * @brief Disables the specified mux configuration
     *
     * @param mux Mux configuration to disable
     */
    void disableInput(ADS1118Mux mux);

    /**
     * @brief Disables all the inputs
     */
    void disableAllInputs();

    /**
     * @brief Enables temperature readings
     */
    void enableTemperature();

    /**
     * @brief Disables temperature readings
     */
    void disableTemperature();

    /**
     * @brief Enables the pull up resistor on dout/miso
     */
    void enablePullUpResistor();

    /**
     * @brief Disables the pull up resistor on dout/miso
     */
    void disablePullUpResistor();

    /**
     * @brief Enables the configuration check after it's writing to the device
     */
    void enableConfigCheck();

    /**
     * @brief Disables the configuration check after it's writing to the device
     */
    void disableConfigCheck();

    /**
     * @brief Reads on the fly the specified input
     *
     * @param mux Mux configuration to read
     * @return Voltage value sampled from the channel in mV
     */
    ADS1118Data readInputAndWait(ADS1118Mux mux);

    /**
     * @brief Reads on the fly the temperature
     *
     * @return Temperature in degree
     */
    TemperatureData readTemperatureAndWait();

    /**
     * @brief Returns the last read voltage value for the specified channel
     */
    ADS1118Data getVoltage(ADS1118Mux mux);

    /**
     * @brief Returns the last temperature value
     */
    TemperatureData getTemperature();

    /**
     * @brief Returns the conversion time in us for the specified channel
     */
    int getConversionTime(int8_t channel);

    /**
     * @brief Writes the temperature configuration and check if it is read back
     * correctly
     *
     * @return True if everything ok
     */
    bool selfTest() override;

private:
    /**
     * @brief Reads the previously configured channel while writing the next
     * enabled configuration.
     *
     * Multiple calls are needed to read all the enabled channels.
     */
    ADS1118Data sampleImpl() override;

    /**
     * @brief Writes the configuration specified, reads the previous written
     * configuration's value and stores it. If enabled checks also that the
     * configuration has been written correctly
     *
     * @param nextChannel Channel number to write the configuration of
     * @param prevChannel Channel number to read the configuration of
     * @return True if everything ok
     */
    void readChannel(int8_t nextChannel, int8_t prevChannel);

    /**
     * @brief Reads on the fly the speficied channel
     *
     * @param channel
     */
    void readChannel(int8_t channel);

    int8_t findNextEnabledChannel(int8_t startChannel);

    const SPISlave spiSlave;
    ADS1118Config baseConfig;

    ///< Read the written configuration on each transaction and checks it
    bool configCheck = false;

    ///< Use `delayUs` instead of `sleep`
    const bool busyWait;

    ADS1118Config channelsConfig[NUM_OF_CHANNELS];  ///< Channels configuration
    ADS1118Data values[NUM_OF_CHANNELS];            ///< Voltage values in mV

    ADS1118Config lastConfig;     ///< Last written configuration
    uint8_t lastConfigIndex = 0;  ///< Last written configuration's index

    ///< Rate of sample calls on which the temperature read
    const uint16_t tempDivider;

    uint16_t sampleCounter = 0;  ///< Counts the number of samples made

    ///< Digit value in mV for each pga configurations
    const float PGA_LSB_SIZE[6] = {0.187,   0.125,    0.0625,
                                   0.03125, 0.015625, 0.0078125};

    ///< Conversion times in us + 100us
    const int CONV_TIME[8] = {125000, 62500, 31250, 15625,
                              7813,   4000,  2106,  1163};

    static constexpr float TEMP_LSB_SIZE  = 0.03125;
    static constexpr uint16_t TEMP_CONFIG = 0xF281;

    /**
     * This masks excludes 2 bits from the configuration, the reserved bit and
     * the sigle shot bit
     */
    static constexpr uint16_t CONFIG_MASK = 0xFE7F;
};