/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Pietro Bortolus
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
#include <drivers/dma/DMA.h>
#include <drivers/spi/SPIDriver.h>
#include <sensors/Sensor.h>

#include "ND015XData.h"

namespace Boardcore
{

class ND015D : public Sensor<ND015XData>
{
public:
    static const char MODEL_NAME[];

    /**
     * The datasheet is unclear about the unit of measure,
     * it could be either psi or inH2O but I believe it's the latter
     */
    enum class FullScaleRange : uint8_t
    {
        FS_1  = 0x02,  // 1.0  psi
        FS_2  = 0x03,  // 2.0  psi
        FS_4  = 0x04,  // 4.0  psi
        FS_5  = 0x05,  // 5.0  psi
        FS_10 = 0x06,  // 10.0 psi
        FS_15 = 0x07,  // 15.0 psi
    };

    /**
     * @brief Converts the FullScale value to its corresponding range.
     *
     * @param fsr FullScale value.
     * @return Pressure range.
     */
    static float rangeToPressure(FullScaleRange fsr);

    enum class IOWatchdogEnable : uint8_t
    {
        DISABLED = 0x00,
        ENABLED  = 0x01,
    };

    enum class BWLimitFilter : uint8_t
    {
        BWL_1   = 0x00,  // 1.0 Hz
        BWL_2   = 0x01,  // 2.0 Hz
        BWL_5   = 0x02,  // 5.0 Hz
        BWL_10  = 0x03,  // 10  Hz
        BWL_20  = 0x04,  // 20  Hz
        BWL_50  = 0x05,  // 50  Hz
        BWL_100 = 0x06,  // 100 Hz
        BWL_200 = 0x07,  // 200 Hz
    };

    enum class NotchEnable : uint8_t
    {
        DISABLED = 0x00,
        ENABLED  = 0x01,
    };

    /**
     * @brief Constructs the default config for the SPI bus.
     *
     * @return The default SPIBusConfig object.
     */
    static SPIBusConfig getDefaultSPIConfig();

    /**
     * @brief Constructor for the ND015D sensor.
     *
     * @param bus SPI bus interface.
     * @param cs Chip select GPIO pin.
     * @param spiConfig SPI bus configuration.
     * @param streamRx Dma receiving stream for the spi bus.
     * @param streamTx Dma transmitting stream for the spi bus.
     * @param timeoutDma Timeout for the dma transactions.
     */
    ND015D(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig,
           DMAStreamGuard* streamRx, DMAStreamGuard* streamTx,
           std::chrono::nanoseconds timeoutDma,
           FullScaleRange fsr   = FullScaleRange::FS_2,
           IOWatchdogEnable iow = IOWatchdogEnable::DISABLED,
           BWLimitFilter bwl    = BWLimitFilter::BWL_200,
           NotchEnable ntc = NotchEnable::ENABLED, uint8_t odr = 0x1C);

    /**
     * @brief Constructor for the ND015D sensor.
     *
     * @param bus SPI bus interface.
     * @param cs Chip select GPIO pin.
     * @param spiConfig SPI bus configuration.
     */
    ND015D(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig,
           FullScaleRange fsr   = FullScaleRange::FS_2,
           IOWatchdogEnable iow = IOWatchdogEnable::DISABLED,
           BWLimitFilter bwl    = BWLimitFilter::BWL_200,
           NotchEnable ntc = NotchEnable::ENABLED, uint8_t odr = 0x1C);

    /**
     * @brief Initializes the sensor.
     *
     * @return Always returns true.
     */
    bool init() override;

    /**
     * @brief Not implemented.
     *
     * @return Always returns true.
     */
    bool selfTest() override;

    /**
     * @brief function to set the output data rate
     *
     * @param odr   output data rate for the sensor,
     *              the actual odr is calculated as
     *              444Hz / odr.
     *              Allowed values are 0x00 to 0xFF,
     *              0x00 will select the auto-select rate mode
     */
    void setOutputDataRate(uint8_t odr);

    /**
     * @brief Sets the full-scale range for the sensor.
     *
     * @param fs Full-scale range.
     */
    void setFullScaleRange(FullScaleRange fsr);

    /**
     * @brief Enables or disables the IO watchdog.
     *
     * @param iow IO watchdog setting.
     */
    void setIOWatchdog(IOWatchdogEnable iow);

    /**
     * @brief Sets the bandwidth limit filter for the sensor.
     *
     * @param bwl Bandwidth limit filter setting.
     */
    void setBWLimitFilter(BWLimitFilter bwl);

    /**
     * @brief Enables or disables the notch filter.
     *
     * @param ntc Notch filter setting.
     */
    void setNotch(NotchEnable ntc);

    /**
     * @brief   Checks if the sensor model matches the expected model.
     *
     * @return  True if the model matches, false otherwise.
     *
     * @warning The function might return false even when it should not as the
     *          SPI transaction sometimes ads some zeroes when it should not.
     *          This is because the sensore requires a clock cycle greater than
     *          the one we can provide
     */
    bool checkModelMatch();

    /**
     * @brief Set the offset of this sensor.
     *        The offset is stored as a int16_t and can be both postive or
     *        negative
     *
     * @param  The value the offset should be set to
     */
    void setOffset(float offset);

    /**
     * @brief Modify the offset of this sensor.
     *        The offset is stored as a int16_t and can be both postive or
     *        negative
     *
     * @param  The value to be added to the current offset
     */
    void updateOffset(float offset);

    /**
     * @brief Get the current offset of this sensor.
     *        The offset is stored as a int16_t and can be both postive or
     *        negative
     */
    float getOffset();

protected:
    ND015XData sampleImpl() override;

private:
    SPISlave slave;
    float range;
    float pressureOffset = 0;

    /**
     * @brief settings for the mode control register,
     *        the initial values are the ones set by default
     *        in the sensor
     *
     * @note  The odr is before the other settings because
     *        the sensors expects the data to be sent MSB first
     */
    struct
    {
        uint8_t odr : 8;           // output data rate
        FullScaleRange fsr : 3;    // full scale range
        IOWatchdogEnable iow : 1;  // IO watchdog enable
        BWLimitFilter bwl : 3;     // bandwidth limit filter
        NotchEnable ntc : 1;       // notch filter enable
    } sensorSettings;

    static_assert(sizeof(sensorSettings) == 2,
                  "sensorSettings size is not 2 bytes");

    struct ND015DDataExtended
    {
        uint16_t pressure;
        uint16_t temperature;
        char model[8];
        uint8_t serial[4];
        uint8_t build[6];
    };

    PrintLogger logger = Logging::getLogger("nd015d");
};

}  // namespace Boardcore
