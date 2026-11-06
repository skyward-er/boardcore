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

namespace Boardcore
{

using namespace AS5047UDefs;

struct AS5047UConfig
{
    UVW uvw                         = UVW::OFF;
    ABI abi                         = ABI::OFF;
    Filter filter                   = Filter::OFF;
    FilterK kMin                    = FilterK::K_0;
    FilterK kMax                    = FilterK::K_0;
    NoiseSet noiseSet               = NoiseSet::FALSE;
    DAECompensation daeCompensation = DAECompensation::OFF;
    DAECStatus daecStatus           = DAECStatus::ON;
    PWM pwm                         = PWM::OFF;
};

/**
 * @note: template is for reading frames (16-bit without crc, 24-bit with crc)
 * writing frames are always 24-bit (32-bit frames are just padded)
 */
template <Frame ReadFrame>
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
     * @note This setting is not persisted after a reset so be sure to call
     it
     * every time
     * @returns true if the angle is reset successfully.
     */
    bool resetAngleZero();

    /**
     * @brief This method enable the Dynamic Angle Error Compensation
     (ANGLECOM
     * will now read compensated angle)
     * @returns true if successfully enabled
     */
    bool enableDAEC();

    /**
     * @brief This method disables the Dynamic Angle Error Compensation
     * (ANGLECOM will now read non-compensated angle)
     * @returns true if successfully disabled
     */
    bool disableDAEC();

protected:
    AS5047UData sampleImpl() override;

private:
    bool initDisable();
    bool initSettings1();
    bool initSettings2();

    /**
     * @brief Selects the source for a read operation in sample().
     * @param daecStatus The source, ANGLECOM to use DAE compensation, ANGLEUNC
     * for the raw uncompensated angle.
     * @returns true if the source is updated correctly.
     */
    bool setDAECStatus(DAECStatus daecStatus);

    /**
     * @brief Writes new value to the register and checks if it was set.
     * Checking is done by reading it again.
     * @param regName register name
     * @param reg register
     * @param value new value to set
     * @param errorOnFail the error to be set to lastError if there was an issue
     * @returns true if the write was successful
     */
    bool writeAndCheck(const std::string& regName,
                       const AS5047UDefs::Registers reg, const uint16_t value,
                       SensorErrors errorOnFail);

    /**
     * @brief This method will just read the error register and log its content.
     * @param readDiagnostic whether to read diagnostic message register.
     */
    void dumpErrorRegister(bool readDiagnostic);

    void writeRegister(Registers target, uint16_t data);
    ReadFrame::Response readRegister(Registers target,
                                     const std::string& regName,
                                     SensorErrors errorOnFail);
    ReadFrame::raw_t transferFrame(ReadFrame::raw_t frame);

    // // void selectActiveInterface(AS5047DDefs::UVWABISelect interface);
    // /**
    //  * @brief This method activates (DAEC_ON) or deactivates (DAEC_OFF)
    //  the
    //  * Dynamic Angle Error Compensation (DAEC).
    //  * @note if DAEC is disabled then the register ANGLECOM (Angle
    //  Compensated)
    //  * will give the Angle without the DAEC
    //  * @param status the status of the DAEC
    //  */
    // void setDAECStatus(AS5047DDefs::DAECStatus status);

    /**
     * @brief The SPI driver used to create SPI Transactions
     */
    SPISlave spiSlave;

    bool initialized;

    AS5047UConfig config;

    PrintLogger logger = Logging::getLogger("as5047u");
};

}  // namespace Boardcore
