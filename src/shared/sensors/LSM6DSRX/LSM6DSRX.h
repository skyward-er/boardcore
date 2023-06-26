/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Fabrizio Monti
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
#include <miosix.h>
#include <sensors/Sensor.h>

#include "LSM6DSRXConfig.h"
#include "LSM6DSRXData.h"
#include "LSM6DSRXDefs.h"

namespace Boardcore
{

/**
 * @brief LSM6DSRX Driver.
 */
class LSM6DSRX : public SensorFIFO<LSM6DSRXData, LSM6DSRXDefs::FIFO_SIZE>
{
public:
    /**
     * @brief LSM6DSRX constructor.
     *
     * @param bus SPI bus.
     * @param csPin SPI chip select pin.
     * @param busConfiguration SPI bus configuration.
     * @param config LSM6DSRX configuration.
     */
    LSM6DSRX(SPIBus& bus, miosix::GpioPin csPin, SPIBusConfig busConfiguration,
             LSM6DSRXConfig& config);

    /**
     * @brief Initialize the sensor.
     */
    bool init() override;

    /**
     * @brief Performs self test for the sensor.
     * @return Return true if the test was successful.
     */
    bool selfTest() override;

    /**
     * @brief Returns data from the sensors.
     */
    LSM6DSRXData getSensorData();

    /**
     * @brief Gather data from FIFO/data registers.
     */
    LSM6DSRXData sampleImpl() override;

private:
    bool isInit = false;
    SPISlave spiSlave;
    LSM6DSRXConfig config;

    float sensitivityAcc = 0.0;  ///< Sensitivity value for the accelerator.
    float sensitivityGyr = 0.0;  ///< Sensitivity value for the gyroscope.

    /**
     * @brief These two values represent the same instant from the two different
     * clocks: the one on the sensor and the one from TimestampTimer. They are
     * used to convert a sensor timestamp to a TimestampTimer one.
     */
    uint64_t timestamp0 = 0;  ///< Timestamp given by TimestampTimer class.
    uint32_t sensorTimestamp0 = 0;  ///< Timestamp given by the sensor.

    float sensorTimestampResolution =
        0.0;  ///< Resolution of the sensor's timestamps in microseconds.

    /**
     * @brief When extracting data from fifo i get data only from one sensor,
     * but the struct LSM6DSRXData is made to have data from both sensors. The
     * idea is to copy the value from the last valid sample for the sensor that
     * hasn't received data.
     */
    LSM6DSRXData lastValidSample;

    /**
     * @brief When extracting samples from fifo data is first read and saved
     * inside this array, then it get processed and stored inside `lastFifo`.
     */
    std::array<LSM6DSRXDefs::RawFifoData, LSM6DSRXDefs::SENSOR_FIFO_SIZE>
        rawFifo;

    PrintLogger logger = Logging::getLogger("lsm6dsrx");

    /**
     * @brief Check who_am_i register for validity.
     *
     * @return Returns false if not valid.
     */
    bool checkWhoAmI();

    /**
     * @brief Performs a really simple reading from the FIFO buffer.
     */
    void readFromFifo();

    /**
     * @brief Utility function to handle the saving of a sample inside the fifo
     * `lastFifo`. This operation could fail if the data to be pushed has the
     * same timestamp of the last value pushed inside `lastFifo` or the
     * timestamp is 0: in this case nothing happens to the fifo, data inside
     * `timeslot` is discarded.
     *
     * @param timeslot The timeslot containing data to be pushed inside the
     * fifo.
     * @param fifoIdx The current index of the fifo, pointing to the current
     * writable cell. Updated if operation is successful, unchanged otherwise.
     */
    void pushIntoFifo(LSM6DSRXDefs::FifoTimeslotData& timeslot,
                      uint16_t& fifoIdx);

    /**
     * @brief Returns the number of unread data in the fifo.
     */
    uint16_t unreadDataInFifo();

    /**
     * @brief Utility for combining two 8 bits numbers in one 16 bits number.
     * @param low Low bits of the 16 bit number.
     * @param high High bits of the 16 bit number.
     */
    int16_t combineHighLowBits(uint8_t low, uint8_t high);

    /**
     * @brief Utility for combining two 8 bits numbers in one 16 bits number.
     * @param low Low bits of the 16 bit number.
     * @param high High bits of the 16 bit number.
     */
    uint16_t combineHighLowBitsUnsigned(uint8_t low, uint8_t high);

    /**
     * @brief Reads 16-bits float data from the specified registers.
     * @param lowReg Register containing the low bits of the output.
     * @param highReg Register containing the high bits of the output.
     * @param sensitivity Sensitivity value for the sample.
     */
    float getAxisData(LSM6DSRXDefs::Registers lowReg,
                      LSM6DSRXDefs::Registers highReg, float sensitivity);

    /**
     * @brief Reads 16-bits data from the specified registers.
     * @param lowReg Register containing the low bits of the output.
     * @param highReg Register containing the high bits of the output.
     */
    int16_t getAxisData(LSM6DSRXDefs::Registers lowReg,
                        LSM6DSRXDefs::Registers highReg);

    /**
     * @brief Initialize the accelerometer.
     */
    void initAccelerometer();

    /**
     * @brief Initialize the gyroscope.
     */
    void initGyroscope();

    /**
     * @brief Initialize fifo.
     */
    void initFifo();

    /**
     * @brief Initialize interrupts.
     */
    void initInterrupts();

    /**
     * @brief Performs self test for the accelerometer.
     * @return Return true if successful.
     */
    bool selfTestAcc();

    /**
     * @brief Performs self test for the gyroscope.
     * @return Return true if successful.
     */
    bool selfTestGyr();

    /**
     * @brief Retrieves data from the accelerometer.
     * @param data The structure where data from the sensor is to be saved.
     */
    void getAccelerometerData(LSM6DSRXData& data);

    /**
     * @brief Retrieves data from the gyroscope.
     * @param data The structure where data from the sensor is to be saved.
     */
    void getGyroscopeData(LSM6DSRXData& data);

    /**
     * @brief Converts timestamp from the value given by the sensor to the one
     * given by TimestampTimer class.
     * @param sensorTimestamp The timestamp from the sensor to be converted into
     * a TimestampTimer one.
     */
    uint64_t convertTimestamp(const uint64_t sensorTimestamp);

    /**
     * @brief Utility to set timestamp values for conversion.
     */
    void correlateTimestamps();

    /**
     * @brief Returns the timestamp from the sensor.
     */
    uint32_t getSensorTimestamp();

    /**
     * @brief Returns the timestamp resolution in milliseconds.
     */
    float getSensorTimestampResolution();

    /**
     * @brief Converts the sample from the sensor's unit of measurement (milli-g
     * for accelerometer data, milli-degree per second for gyroscope data) to
     * our units (meters per second squared and radiants per second).
     *
     * @param sample The sample to be converted.
     */
    void convertSampleMeasurementUnit(LSM6DSRXData& sample);
};

}  // namespace Boardcore
