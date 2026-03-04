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

    static SPIBusConfig getDefaultSPIConfig();

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

    /**
     * @brief This method will read the current angle and will set it as the
     * zero of the sensor
     * @note This setting is not persisted after a reset so be sure to call it
     * every time
     */
    void resetAngleZero();

    /**
     * @brief This method enable the Dynamic Angle Error Compensation (ANGLECOM
     * will now read compensated angle)
     */
    void enableDAEC() { setDAECStatus(AS5047DDefs::DAECStatus::DAEC_ON); }

    /**
     * @brief This method disables the Dynamic Angle Error Compensation
     * (ANGLECOM will now read non-compensated angle)
     */
    void disableDAEC() { setDAECStatus(AS5047DDefs::DAECStatus::DAEC_ON); }

    /**
     * @brief This method will set the data source for the ANGLECOM register:
     * DAECANG for dynamically compensated angle, CORDICANG for the raw angle
     * @param dataSource The source that will fill ANGLECOM
     */
    void setDataSource(AS5047DDefs::DataSelect dataSource);

protected:
    AS5047DData sampleImpl() override;

private:
    /**
     * @brief This method will just read the error register and log its content.
     */
    void dumpErrorRegister();

    /**
     * This class is used to represent the result of a read from the sensor.
     * It is useful since the packet sent by the sensor can also tell if there
     * was a error or not.
     */
    typedef struct
    {
        uint16_t data;  ///< The data read from the sensor, valid if hasError()
                        ///< returns false
        AS5047DDefs::Error error;  ///< Contains the status of the transaction,
                                   ///< it is set to NONE if there was no errors

        /**
         * @brief This method returns if there was an error or not during the
         * transaction
         */
        bool hasError() { return error != AS5047DDefs::Error::NONE; }
    } ReadResult;

    // void setPWMStatus(AS5047DDefs::PWMSelect pwmEnable);
    // void selectActiveInterface(AS5047DDefs::UVWABISelect interface);
    /**
     * @brief This method activates (DAEC_ON) or deactivates (DAEC_OFF) the
     * Dynamic Angle Error Compensation (DAEC).
     * @note if DAEC is disabled then the register ANGLECOM (Angle Compensated)
     * will give the Angle without the DAEC
     * @param status the status of the DAEC
     */
    void setDAECStatus(AS5047DDefs::DAECStatus status);

    /**
     * @brief This method will write a 16 bit register address packet with a 14
     * bit data packet and will also calculate the parity bit of both packets as
     * required by the sensor
     * @param reg The register to be written
     * @param data 14 bit data (the two msb will be erased)
     */
    void writeRegister(AS5047DDefs::Registers reg, uint16_t data);

    /**
     * @brief This method is used to read the content from a register. Since
     * this sensor implements an error checking protocolo the result of this
     * call should be checked for error with the given method.
     * @param reg The register to be read
     * @returns ReadResult object containing the data or the error that happened
     * during the transaction
     */
    ReadResult readRegister(AS5047DDefs::Registers reg);

    /**
     * @brief This method calculates the parity of the given data.
     * @param data The data that the parity will be calculated on
     * @returns The parity bit (in the lsb)
     */
    uint16_t getParity(uint16_t data);

    /**
     * @brief This method is just a helper to facilitate the logging of any
     * error happening during the transaction and eventually also it will dump
     * the error register
     * @param regName The name of the register in human readable format
     * @param error The error given in the last transaction
     * @param shorErrorReg wether to dump or not the error register present on
     * the sensor (default=true)
     */
    void logReadRegisterError(std::string regName, AS5047DDefs::Error error,
                              bool showErrorReg = true);

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
