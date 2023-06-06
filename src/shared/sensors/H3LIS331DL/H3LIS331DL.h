/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Radu Raul
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
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/Sensor.h>

#include "H3LIS331DLData.h"
#include "H3LIS331DLDefs.h"

namespace Boardcore
{

/**
 * Driver for H3LIS331DL, a 3-Axis, high g Accelerometer Sensor made by
 * STMicroelectronics.
 */
class H3LIS331DL : public Sensor<H3LIS331DLData>
{

public:
    /**
     * @brief Creates an instance of an H3LIS331DL sensor
     *
     * @param spiBus The SPI bus the sensor is connected to
     * @param cs The Chip Select GPIO
     * @param odr Sensor's Output Data Rate (See datasheet)
     * @param bdu Sensor's Block Data Update (See datasheet)
     * @param fs Sensor's Full Scale Range (See datasheet)
     */
    H3LIS331DL(SPIBusInterface& spiBus, miosix::GpioPin cs,
               H3LIS331DLDefs::OutputDataRate odr,
               H3LIS331DLDefs::BlockDataUpdate bdu,
               H3LIS331DLDefs::FullScaleRange fs);

    /**
     * @brief Creates an instance of an H3LIS331DL sensor
     *
     * @param spiBus The SPI bus the sensor is connected to
     * @param cs  The Chip Select GPIO
     * @param cfg SPI Bus Configuration
     * @param odr Sensor's Output Data Rate (See datasheet)
     * @param bdu Sensor's Block Data Update (See datasheet)
     * @param fs Sensor's Full Scale Range (See datasheet)
     */
    H3LIS331DL(SPIBusInterface& spiBus, miosix::GpioPin cs, SPIBusConfig cfg,
               H3LIS331DLDefs::OutputDataRate odr,
               H3LIS331DLDefs::BlockDataUpdate bdu,
               H3LIS331DLDefs::FullScaleRange fs);

    /**
     * @brief Initializes the H3LIS331DL
     *
     * The init function writes the configuration to the configuration
     * registers so no further writes are needed.
     *
     * @returns True if the initialization was OK. Returns False otherwise (use
     * getLastError()) method to get information about the last error.
     */
    bool init();

    /**
     * @brief Samples data from the register.
     *
     * This method reads the data from the 3 pairs (one pair for each axis) of
     * 8bit registers and convert it to floating point value based on the Full
     * Scale Range specified at construction time.
     * The data is returned in the H3LIS331DLData struct correlated to a
     * timestamp of when the data was sampled.
     *
     * @returns A copy of an instance of H3LIS331DLData.
     */
    H3LIS331DLData sampleImpl() override;

    /**
     * @brief This method does nothing as no self test is implemented in the
     * sensor
     *
     * @returns True always.
     */
    bool selfTest();

private:
    /**
     * @brief The SPI driver used to create SPI Transactions
     */
    SPISlave spi;

    /**
     * @brief The OutputDataRate that is set to the sensor.
     *
     * Default: 50 HZ (@see OutputDataRate::ODR_50)
     *
     * Note: setting the ODR also sets the PowerMode.
     * If the ODR is less than ODR_50 low PowerMode is set.
     * Otherwise normal PowerMode is set.
     */
    H3LIS331DLDefs::OutputDataRate odr;

    /**
     * @brief The BlockDataUpdate that is set to the sensor.
     *
     * Default: Continuos Update (@see
     * H3LIS331DL::BlockDataUpdate::BDU_CONTINUOS_UPDATE)
     */
    H3LIS331DLDefs::BlockDataUpdate bdu;

    /**
     * @brief The Full Scale Range set to the sensor.
     *
     * Default: +-100 (@see H3LIS331DL::FullScaleRange::FS_100)
     *
     * Note: setting the FSR also changes the sensitivity of the sensor.
     */
    H3LIS331DLDefs::FullScaleRange fs;

    /**
     * @brief True if the sensor is already initialized, False otherwise.
     *
     * Note: This is only changed by init, read by init and sampleImpl.
     */
    bool initialized;

    PrintLogger logger = Logging::getLogger("h3lis331dl");
};

}  // namespace Boardcore
