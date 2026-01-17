/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Raul Radu
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

#include "AS5047DData.h"
#include "AS5047DDefs.h"

namespace Boardcore
{

struct AS5047DSPIConfig
{
    AS5047DDefs::UVWABISelect dataInterface;
    AS5047DDefs::ABIRotationDirection rotationDirection;
    AS5047DDefs::PWMSelect pwmEnabled;
    AS5047DDefs::UVWPolePairs uvwPolePairs;
    AS5047DDefs::HysteresisConfiguration hystConfig;
    AS5047DDefs::DataSelect dataType;
    AS5047DDefs::DAECStatus daecEnabled;
    AS5047DDefs::ABIResolution abiResolution;
};

class AS5047DSPI : public Sensor<AS5047DData>
{
public:
    AS5047DSPI(SPIBusInterface& spiBus, miosix::GpioPin cs, SPIBusConfig spiCfg,
               AS5047DSPIConfig sensorCfg);

    AS5047DSPI(SPIBusInterface& spiBus, miosix::GpioPin cs,
               AS5047DSPIConfig sensorCfg);
    bool init() override;
    bool selfTest() override;

    // void useUVWInterface();
    // void useABIInterface();

    // void enablePWM();
    // void disablePWM();

    // void setABIResolution(AS5047DDefs::ABIResolution abiResolution);
    // void setABIRotationDirection(AS5047DDefs::ABIRotationDirection rdir);
    // void switchABIRotationDirection();
    // void setABIHysteresisConfiguration(
    //     AS5047DDefs::HysteresisConfiguration config);

    // void setUVWPolePairsNumber(AS5047DDefs::UVWPolePairs pairs);

    void enableDAEC() { setDAECStatus(AS5047DDefs::DAECStatus::DAEC_ON); }
    void disableDAEC() { setDAECStatus(AS5047DDefs::DAECStatus::DAEC_ON); }
    void setDataSource(AS5047DDefs::DataSelect dataSource);

protected:
    AS5047DData sampleImpl() override;

private:
    void dumpErrorRegister();
    typedef struct
    {
        uint16_t data;
        AS5047DDefs::Error error;

        bool hasError() { return error != AS5047DDefs::Error::NONE; }
    } ReadResult;

    // void setPWMStatus(AS5047DDefs::PWMSelect pwmEnable);
    // void selectActiveInterface(AS5047DDefs::UVWABISelect interface);
    void setDAECStatus(AS5047DDefs::DAECStatus status);
    void writeRegister(AS5047DDefs::Registers reg, uint16_t data);
    ReadResult readRegister(AS5047DDefs::Registers reg);
    SPIBusConfig getDefaultSPIConfig();
    uint16_t getParity(uint16_t data);

    void logReadRegisterError(std::string regName, AS5047DDefs::Error error);

    /**
     * @brief The SPI driver used to create SPI Transactions
     */
    SPISlave spiSlave;

    /// true if init completes successfully, false otherwise
    bool initialized;

    AS5047DSPIConfig config;

    PrintLogger logger = Logging::getLogger("as5047dspi");
};

}  // namespace Boardcore
