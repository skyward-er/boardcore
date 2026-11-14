
#pragma once

#include <array>
#include <cstdint>

#include <diagnostic/PrintLogger.h>
#include <drivers/spi/SPIDriver.h>
#include <miosix.h>
#include <sensors/Sensor.h>

#include "LSM6DSV320XConfig.h"
#include "LSM6DSV320XData.h"
#include "LSM6DSV320XDefs.h"

namespace Boardcore
{

/**
 * @brief LSM6DSV320X Driver.
 */
class LSM6DSV320X
    : public SensorFIFO<LSM6DSV320XData, LSM6DSV320XDefs::FIFO_SIZE>
{
public:
    /**
     * @brief LSM6DSV320X constructor.
     *
     * @param bus SPI bus.
     * @param csPin SPI chip select pin.
     * @param busConfiguration SPI bus configuration.
     * @param config LSM6DSV320X configuration.
     */
    LSM6DSV320X(SPIBusInterface& bus, miosix::GpioPin csPin,
                SPIBusConfig busConfiguration, LSM6DSV320XConfig& config);

    /**
     * @brief Initialize the sensor.
     */
    bool init() override;

    /**
     * @brief Performs self test for the sensor.
     * @return True if the test was successful.
     */
    bool selfTest() override;

    /**
     * @brief Returns data from the sensors.
     */
    LSM6DSV320XData getSensorData();

protected:
    /**
     * @brief Gather data from FIFO/data registers.
     */
    LSM6DSV320XData sampleImpl() override;

private:
    bool isInit = false;

    SPISlave spiSlave;
    LSM6DSV320XConfig config;

    // Sensor sensitivities
    float sensitivityLowG = 0.0;
    float sensitivityHighG = 0.0;
    float sensitivityGyr = 0.0;

    /**
     * @brief Correlation between the sensor clock and TimestampTimer.
     */
    uint64_t timestamp0 = 0;
    uint32_t sensorTimestamp0 = 0;

    float sensorTimestampResolution = 0.0;

    /**
     * @brief Stores the last valid sample for each sensor channel.
     */
    LSM6DSV320XData lastValidSample;

    /**
     * @brief Temporary buffer containing raw FIFO records.
     */
    std::array<LSM6DSV320XDefs::RawFifoData,
               LSM6DSV320XDefs::SENSOR_FIFO_SIZE> rawFifo;

    PrintLogger logger = Logging::getLogger("lsm6dsv320x");

    /**
     * @brief Checks the WHO_AM_I register.
     */
    bool checkWhoAmI();

    /**
     * @brief Reads raw records from the hardware FIFO.
     */
    void readFromFifo();

    /**
     * @brief Returns the number of unread records in the hardware FIFO.
     */
    uint16_t unreadDataInFifo();

    /**
     * @brief Combines two 8-bit values into a signed 16-bit value.
     */
    int16_t combineHighLowBits(uint8_t low, uint8_t high);

    /**
     * @brief Combines two 8-bit values into an unsigned 16-bit value.
     */
    uint16_t combineHighLowBitsUnsigned(uint8_t low, uint8_t high);

    /**
     * @brief Reads a signed 16-bit value from two consecutive registers.
     */
    int16_t readInt16(LSM6DSV320XDefs::Registers lowReg,
                      LSM6DSV320XDefs::Registers highReg);

    /**
     * @brief Initializes the Low-G accelerometer.
     */
    void initLowG();

    /**
     * @brief Initializes the High-G accelerometer.
     */
    void initHighG();

    /**
     * @brief Initializes the gyroscope.
     */
    void initGyroscope();

    /**
     * @brief Initializes the FIFO.
     */
    void initFifo();

    /**
     * @brief Initializes interrupts.
     */
    void initInterrupts();

    /**
     * @brief Performs self test for the Low-G accelerometer.
     */
    bool selfTestLowG();

    /**
     * @brief Performs self test for the gyroscope.
     */
    bool selfTestGyr();

    /**
     * @brief Retrieves Low-G accelerometer data.
     */
    void getLowGData(LSM6DSV320XData& data);

    /**
     * @brief Retrieves High-G accelerometer data.
     */
    void getHighGData(LSM6DSV320XData& data);

    /**
     * @brief Retrieves gyroscope data.
     */
    void getGyroscopeData(LSM6DSV320XData& data);

    /**
     * @brief Retrieves temperature data.
     */
    void getTemperatureData(LSM6DSV320XData& data);

    /**
     * @brief Converts a sensor timestamp to a TimestampTimer timestamp.
     */
    uint64_t convertTimestamp(const uint64_t sensorTimestamp);

    /**
     * @brief Sets the timestamp correlation values.
     */
    void correlateTimestamps();

    /**
     * @brief Returns the sensor timestamp.
     */
    uint32_t getSensorTimestamp();

    /**
     * @brief Returns the sensor timestamp resolution in milliseconds.
     */
    float getSensorTimestampResolution();
};

}  // namespace Boardcore
