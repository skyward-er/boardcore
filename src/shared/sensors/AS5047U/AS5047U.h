/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Aldas Lenkšas
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

#include "AS5047UData.h"
#include "AS5047UDefs.h"
#include "AS5047UFrames.h"

namespace Boardcore
{

/**
 * @brief Configuration values for AS5047U analog sensor given with the
 * corresponding default values.
 *
 * The values for the Adaptive Noise Filter (kMin and kMax) should be set
 * depending on the application.
 *
 * The noiseSet value should be set depending on the operation mode (3.3V or
 * 5V), and on possible temperatures. See the description on
 * AS5047UDefs::NoiseSet.
 */
struct AS5047UConfig
{
    AS5047UDefs::Filter filterOn        = AS5047UDefs::Filter::ON;
    AS5047UDefs::FilterK kMin           = AS5047UDefs::FilterK::K_0;
    AS5047UDefs::FilterK kMax           = AS5047UDefs::FilterK::K_6;
    AS5047UDefs::NoiseSet noiseSet      = AS5047UDefs::NoiseSet::FALSE;
    AS5047UDefs::DAECompensation daecOn = AS5047UDefs::DAECompensation::ON;
    AS5047UDefs::DAECSource daecSource  = AS5047UDefs::DAECSource::ANGLECOM;
};

/**
 * AS5047U sensor is implement to read the angle data.
 *
 * The communication with the sensor is done with SPI protocol. The sensor
 * supports three types of frames: 16-bit, 24-bit, and 32-bit frames. 32-bit
 * frames are not supported by this implementation, as 24-bit frames are better.
 *
 * 16-bit frames do not allow for SPI write commands. Because of that writes are
 * always done with 24-bit frames. For the read commands, both 16-bit and 24-bit
 * frames can be chosen by selecting the templated frame type for the object of
 * this class. 24-bit frames use CRC (Cyclic Redundancy Check), while the 16-bit
 * ones do not.
 *
 * The sensor gives the result to the read command on the subsequent SPI read
 * which means that the result of reading the registers is delayed. To account
 * for that, while setting the configuration if the reads are needed, two reads
 * are done. However, note that for sampling the sensor, only one SPI transfer
 * is done. That means the first sensor reading sample is invalid, and all the
 * subsequent ones will be returning the values delayed by one sample call.
 */
template <AS5047UFrames::Frame ReadFrame>
class AS5047U : public Sensor<AS5047UData>
{
public:
    AS5047U(SPIBusInterface& spiBus, miosix::GpioPin cs, SPIBusConfig& spiCfg,
            AS5047UConfig& sensorCfg);

    AS5047U(SPIBusInterface& spiBus, miosix::GpioPin cs,
            AS5047UConfig& sensorCfg);

    bool init() override;

    bool selfTest() override;

    static SPIBusConfig getDefaultSPIConfig();

    /**
     * @brief This method will read the current angle and will set it as the
     * zero of the sensor
     * @note This setting is not persisted after a reset so be sure to call it
     * every time
     * @returns true if the angle is reset successfully.
     */
    bool resetAngleZero();

    /**
     * @brief This method enable the Dynamic Angle Error Compensation. The
     * sensor will return compensated angles when sampling.
     * @returns true if successfully enabled
     */
    bool enableDAEC();

    /**
     * @brief This method disables the Dynamic Angle Error Compensation. The
     * sensor will return uncompensated angles when sampling.
     * @returns true if successfully disabled
     */
    bool disableDAEC();

protected:
    AS5047UData sampleImpl() override;

private:
    bool initDisableRegister();
    bool initSettings1Register();
    bool initSettings2Register();

    /**
     * @brief Turns on/off the Dynamic Angle Error Compensation, and changes the
     * source for ANGLECOM register.
     * @param daecOn Whether DAEC is on.
     * @param daecSource The source, ANGLECOM to use DAE compensation, ANGLEUNC
     * for the raw uncompensated angle.
     * @returns true if the source is updated correctly.
     */
    bool setDAECStatus(AS5047UDefs::DAECompensation daecOn,
                       AS5047UDefs::DAECSource daecSource);

    /**
     * @brief Writes new value to the register and checks if it was set.
     * Checking is done by reading it again.
     * @param reg register
     * @param value new value to set
     * @param errorOnFail the error to be set to lastError if there was an issue
     * @returns true if the write was successful
     */
    bool writeAndCheck(const AS5047UDefs::Registers reg, const uint16_t value,
                       SensorErrors errorOnFail);

    /**
     * @brief This method will just read the error register and log its content.
     * @param readDiagnostic whether to read diagnostic message register.
     */
    void dumpErrorRegister(bool readDiagnostic);

    /**
     * @brief Checks if there was an error or a warning at the response given as
     * an SPI frame. If yes, will log the corresponding problem.
     */
    void checkResponseErrors(AS5047UDefs::Registers target,
                             typename ReadFrame::Response resp,
                             SensorErrors errorOnFail);

    void writeRegister(AS5047UDefs::Registers target, uint16_t data);

    typename ReadFrame::Response readRegister(AS5047UDefs::Registers target,
                                              SensorErrors errorOnFail);

    /**
     * @brief Transfers the given frame as an SPI transaction by splitting it
     * into bytes.
     */
    typename ReadFrame::raw_t transferFrame(typename ReadFrame::raw_t frame);

    /**
     * @brief The SPI driver used to create SPI Transactions.
     */
    SPISlave spiSlave;

    bool initialized;

    /**
     * @brief Whether the sample was already taken.
     * This is needed because SPI read only returns value to read
     * on the next SPI transaction with the next read command.
     */
    bool isFirstSample = true;

    AS5047UConfig config;

    PrintLogger logger = Logging::getLogger("as5047u");
};

}  // namespace Boardcore

#include "AS5047U.tpp"

