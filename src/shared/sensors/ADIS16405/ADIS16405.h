/* ADIS16405 Driver
 *
 * Copyright (c) 2018-2019 Skyward Experimental Rocketry
 * Authors: Nuno Barcellos
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef ADIS16405_H
#define ADIS16405_H

#include <drivers/BusTemplate.h>
#include "sensors/Sensor.h"
#include "ADIS16405Data.h"

template <typename BusSPI, typename rstPin>
class ADIS16405 : public AccelSensor,
                  public GyroSensor,
                  public CompassSensor,
                  public TemperatureSensor
{
public:
    ADIS16405(uint8_t gyroFullScale)
    {
        gyroFS = gyroFullScale;
    }
    virtual ~ADIS16405() {}

    std::vector<SPIRequest> buildDMARequest() override
    {
        // clang-format off
        std::vector<uint8_t> v = 
        { 
            ADIS_GLOB_CMD, 0, // System command
            0,0,  //  Power supply measurement
            0,0,  //  X-axis gyroscope output
            0,0,  //  Y-axis gyroscope output
            0,0,  //  Z-axis gyroscope output
            0,0,  //  X-axis accelerometer output
            0,0,  //  Y-axis accelerometer output
            0,0,  //  Z-axis accelerometer output
            0,0,  //  X-axis magnetometer measurement
            0,0,  //  Y-axis magnetometer measurement
            0,0,  //  Z-axis magnetometer measurement
            0,0,  //  Temperature output
            0,0,  //  Auxiliary ADC measurement
        };
        // clang-format on

        return {SPIRequest(0, BusSPI::getCSPin(), v)};
    }

    bool init() override
    {
        // Ensure right SPI frequency
        // Burst mode SPI < 1 MHz
        // Low power mode SPI < 300 kHz

        rstPin::mode(miosix::Mode::OUTPUT);
        rstPin::high(); // Turn on the device

        miosix::delayMs(220); // 220 ms start-up time 

        uint16_t product_id = readReg(ADIS_PRODUCT_ID);
        if (product_id != product_id_value)
        {
            last_error = ERR_NOT_ME;
            return false;
        }

        //TODO: test it -- config the dynamic range
        uint16_t sensorAvg = readReg(ADIS_SENS_AVG);
        writeReg(ADIS_SENS_AVG, sensorAvg | (1 << (8+gyroFS)));


        return true;
    }

    void readTest()
    {   
        // uint16_t product_id = readReg(ADIS_PRODUCT_ID);
        // printf("product_id: %d\n", product_id);

        // float accel = normalizeAccel(signExtend(readReg(ADIS_YACCL_OUT) & 0x3FFF, 14));
        // printf("temperature: %f\n", accel);
        
        // float temperature = normalizeTemp(signExtend(readReg(ADIS_TEMP_OUT) & 0x0FFF, 12));
        // printf("temperature: %f\n", temperature);
        
        // float voltage = normalizePower(readReg(ADIS_SUPPLY_OUT) & 0x3FFF);
        // printf("voltage: %f\n", voltage);

        // writewriteReg(ADIS_ZMAGN_SIF,0x0800);
        // miosix::delayMs(1);
        // uint16_t reg_value = readReg(ADIS_ZMAGN_SIF);
        // printf("reg value: %d\n", reg_value);
        // miosix::delayMs(200);
    }

    void burstTest()
    {
        ADIS16405Data burstData;
        burstDataCollect(&burstData);
        // printf("%f\t",normalizeSupply(burstData.supply_out));
        // printf("%f\t",normalizeGyro(burstData.xgyro_out));
        // printf("%f\t",normalizeGyro(burstData.ygyro_out));
        printf("%f\t",normalizeGyro(burstData.zgyro_out));
        // printf("%f\t",normalizeAccel(burstData.xaccl_out));
        // printf("%f\t",normalizeAccel(burstData.yaccl_out));
        // printf("%f\t",normalizeAccel(burstData.zaccl_out));
        // printf("%f\t",normalizeMagneto(burstData.xmagn_out));
        // printf("%f\t",normalizeMagneto(burstData.ymagn_out));
        // printf("%f\t",normalizeMagneto(burstData.zmagn_out));
        // printf("%f\t",normalizeTemp(burstData.temp_out));
        // printf("%f\t",normalizeADC(burstData.aux_adc));
        printf("\n");
    }

    // Exercises all inertial sensors, measures each response,
    // makes pass/fail decisions, and reports them to error flags
    // in the DIAG_STAT register
    bool selfTest() override
    {
        // DIAG_STAT clears after each read so we read to clear it
        uint16_t diagstat = readReg(ADIS_DIAG_STAT);

        uint16_t msc = readReg(ADIS_MSC_CTRL);
        writeReg(ADIS_MSC_CTRL, msc | 1 << 10);
        // msc = readReg(ADIS_MSC_CTRL);
        
        // writeReg(ADIS_MSC_CTRL, 0x3504); // Config and start self test
        
        // MSC_CTRL[10] resets itself to 0 after completing the self test routine
        do
        {
            msc = readReg(ADIS_MSC_CTRL);
        } while (msc & 1 << 10);

        diagstat = readReg(ADIS_DIAG_STAT);
        return (diagstat == 0); // All bits are cleared if the test was successful
    }

    void onDMAUpdate(const SPIRequest& req) override
    {
        const std::vector<uint8_t>& r = req.readResponseFromPeripheral();

        uint8_t raw_data[sizeof(ADIS16405Data)];        
        memcpy(&raw_data, &(r[2]),sizeof(ADIS16405Data));

        for(uint8_t i = 0; i < sizeof(ADIS16405Data); i++)
            raw_data[i] = fromBigEndian16(raw_data[i]);

        ADIS16405Data data;
        parseBurstData(raw_data,&data);

        mLastGyro.setX(normalizeGyro(data.xgyro_out));
        mLastGyro.setY(normalizeGyro(data.ygyro_out));
        mLastGyro.setZ(normalizeGyro(data.zgyro_out));

        mLastAccel.setX(normalizeAccel(data.xaccl_out));
        mLastAccel.setY(normalizeAccel(data.yaccl_out));
        mLastAccel.setZ(normalizeAccel(data.zaccl_out));

        mLastCompass.setX(normalizeMagneto(data.xmagn_out));
        mLastCompass.setY(normalizeMagneto(data.ymagn_out));
        mLastCompass.setZ(normalizeMagneto(data.zmagn_out));

        mLastTemp = normalizeTemp(data.temp_out);
    }

    bool onSimpleUpdate() override
    { 
        ADIS16405Data data;
        burstDataCollect(&data);

        mLastGyro.setX(normalizeGyro(data.xgyro_out));
        mLastGyro.setY(normalizeGyro(data.ygyro_out));
        mLastGyro.setZ(normalizeGyro(data.zgyro_out));

        mLastAccel.setX(normalizeAccel(data.xaccl_out));
        mLastAccel.setY(normalizeAccel(data.yaccl_out));
        mLastAccel.setZ(normalizeAccel(data.zaccl_out));

        mLastCompass.setX(normalizeMagneto(data.xmagn_out));
        mLastCompass.setY(normalizeMagneto(data.ymagn_out));
        mLastCompass.setZ(normalizeMagneto(data.zmagn_out));

        mLastTemp = normalizeTemp(data.temp_out);

        return true;
    }

    enum gyroFullScale
    {
        GYRO_FS_75 = 0,  // 75°/sec
        GYRO_FS_150 = 1, // 150°/sec
        GYRO_FS_300 = 2  // 300°/sec (default condition)
    };

private:
    // There's a typo in the datasheet saying the value is 0x4105,
    // when it's 0x4015 (== 16405)
    constexpr static uint16_t product_id_value = 0x4015;

    constexpr static float gyroFSMAP[] = {0.25, 0.5, 1};

    uint8_t gyroFS;

    // ADIS Register map
    enum adis_regaddr : uint8_t
    {   
        // clang-format off
        // Name           address    default function
        ADIS_FLASH_CNT  = 0x00,  //  N/A     Flash memory write count
        ADIS_SUPPLY_OUT = 0x02,  //  N/A     Power supply measurement
        ADIS_XGYRO_OUT  = 0x04,  //  N/A     X-axis gyroscope output
        ADIS_YGYRO_OUT  = 0x06,  //  N/A     Y-axis gyroscope output
        ADIS_ZGYRO_OUT  = 0x08,  //  N/A     Z-axis gyroscope output
        ADIS_XACCL_OUT  = 0x0A,  //  N/A     X-axis accelerometer output
        ADIS_YACCL_OUT  = 0x0C,  //  N/A     Y-axis accelerometer output
        ADIS_ZACCL_OUT  = 0x0E,  //  N/A     Z-axis accelerometer output
        ADIS_XMAGN_OUT  = 0x10,  //  N/A     X-axis magnetometer measurement
        ADIS_YMAGN_OUT  = 0x12,  //  N/A     Y-axis magnetometer measurement
        ADIS_ZMAGN_OUT  = 0x14,  //  N/A     Z-axis magnetometer measurement
        ADIS_TEMP_OUT   = 0x16,  //  N/A     Temperature output
        ADIS_AUX_ADC    = 0x18,  //  N/A     Auxiliary ADC measurement
        ADIS_XGYRO_OFF  = 0x1A,  //  0x0000  X-axis gyroscope bias offset factor
        ADIS_YGYRO_OFF  = 0x1C,  //  0x0000  Y-axis gyroscope bias offset factor
        ADIS_ZGYRO_OFF  = 0x1E,  //  0x0000  Z-axis gyroscope bias offset factor
        ADIS_XACCL_OFF  = 0x20,  //  0x0000  X-axis acceleration bias offset factor
        ADIS_YACCL_OFF  = 0x22,  //  0x0000  Y-axis acceleration bias offset factor
        ADIS_ZACCL_OFF  = 0x24,  //  0x0000  Z-axis acceleration bias offset factor
        ADIS_XMAGN_HIF  = 0x26,  //  0x0000  X-axis magnetometer, hard-iron factor
        ADIS_YMAGN_HIF  = 0x28,  //  0x0000  Y-axis magnetometer, hard-iron factor
        ADIS_ZMAGN_HIF  = 0x2A,  //  0x0000  Z-axis magnetometer, hard-iron factor
        ADIS_XMAGN_SIF  = 0x2C,  //  0x0800  X-axis magnetometer, soft-iron factor
        ADIS_YMAGN_SIF  = 0x2E,  //  0x0800  Y-axis magnetometer, soft-iron factor
        ADIS_ZMAGN_SIF  = 0x30,  //  0x0800  Z-axis magnetometer, soft-iron factor
        ADIS_GPIO_CTRL  = 0x32,  //  0x0000  Auxiliary digital input/output control
        ADIS_MSC_CTRL   = 0x34,  //  0x0006  Miscellaneous control
        ADIS_SMPL_PRD   = 0x36,  //  0x0001  Internal sample period (rate) control
        ADIS_SENS_AVG   = 0x38,  //  0x0402  Dynamic range and digital filter control
        ADIS_SLP_CNT    = 0x3A,  //  0x0000  Sleep mode control
        ADIS_DIAG_STAT  = 0x3C,  //  0x0000  System status
        ADIS_GLOB_CMD   = 0x3E,  //  0x0000  System command
        ADIS_ALM_MAG1   = 0x40,  //  0x0000  Alarm 1 amplitude threshold
        ADIS_ALM_MAG2   = 0x42,  //  0x0000  Alarm spi_master_xact_data* caller,
                                 //          spi_master_xact_data* spi_xact, void*
                                 //          data2 amplitude threshold
        ADIS_ALM_SMPL1  = 0x44,  //  0x0000  Alarm 1 sample size
        ADIS_ALM_SMPL2  = 0x46,  //  0x0000  Alarm 2 sample size
        ADIS_ALM_CTRL   = 0x48,  //  0x0000  Alarm control
        ADIS_AUX_DAC    = 0x4A,  //  0x0000  Auxiliary DAC data
        //              = 0x4C to 0x55       Reserved
        ADIS_PRODUCT_ID = 0x56   //  0x4015  Product identifier
        // clang-format on
    };

    uint16_t readReg(adis_regaddr addr)
    {
        uint8_t rxbuf[2];
        BusSPI::write(addr,0);
        miosix::delayUs(9); // This delay should be 75 us for low power mode
        BusSPI::read(rxbuf, 2);
        miosix::delayUs(9); // This delay should be 75 us for low power mode
        return rxbuf[0] << 8 | rxbuf[1];
    }

    void writeReg(adis_regaddr addr, uint16_t value)
    {
        BusSPI::write((addr+1) | 0x80, (uint8_t)(value >> 8));
        BusSPI::write(addr | 0x80, (uint8_t) value);
    }

    void burstDataCollect(ADIS16405Data* data)
    {   
        uint8_t raw_data[sizeof(ADIS16405Data)];
        BusSPI::read16((ADIS_GLOB_CMD) << 8, raw_data, sizeof(ADIS16405Data));
        parseBurstData(raw_data, data);
    }

    void parseBurstData(uint8_t* raw, ADIS16405Data* data)
    {
        // TODO: check nd and ea bits
        data->supply_out = (raw[0] << 8 | raw[1]) & 0x3FFF;
        data->xgyro_out  = signExtend((raw[2] << 8 | raw[3]) & 0x3FFF, 14);
        data->ygyro_out  = signExtend((raw[4] << 8 | raw[5]) & 0x3FFF, 14);
        data->zgyro_out  = signExtend((raw[6] << 8 | raw[7]) & 0x3FFF, 14);
        data->xaccl_out  = signExtend((raw[8] << 8 | raw[9]) & 0x3FFF, 14);
        data->yaccl_out  = signExtend((raw[10] << 8 | raw[11]) & 0x3FFF, 14);
        data->zaccl_out  = signExtend((raw[12] << 8 | raw[13]) & 0x3FFF, 14);
        data->xmagn_out  = signExtend((raw[14] << 8 | raw[15]) & 0x3FFF, 14);
        data->ymagn_out  = signExtend((raw[16] << 8 | raw[17]) & 0x3FFF, 14);
        data->zmagn_out  = signExtend((raw[18] << 8 | raw[19]) & 0x3FFF, 14);
        data->temp_out   = signExtend((raw[20] << 8 | raw[21]) & 0x0FFF, 12);
        data->aux_adc    = (raw[22] << 8 | raw[23]) & 0x0FFF;
    }

    int16_t signExtend(uint16_t val, uint8_t bits)
    {
        if ((val & (1 << (bits - 1))) != 0)
        {
            val = val - (1 << bits);
        }
        return val;
    }

    inline float normalizeSupply(uint16_t val)
    {
        return static_cast<float>(val) * 2.418e-3f; // [V]
    }

    inline float normalizeAccel(int16_t val)
    {
        return static_cast<float>(val) * 3.33e-3f * EARTH_GRAVITY; // [g]
    }

    inline float normalizeGyro(int16_t val)
    {
        return static_cast<float>(val) * 0.05f * gyroFSMAP[gyroFS] *
                DEGREES_TO_RADIANS; // [rad/s]
    }

    inline float normalizeMagneto(int16_t val)
    {
        return static_cast<float>(val) * 0.5e-3f; // [gausss]
    }

    inline float normalizeTemp(int16_t val)
    {
        return static_cast<float>((val) * 0.14f + 25); // [C deg]
    }

    inline float normalizeADC(uint16_t val)
    {
        return static_cast<float>(val) * 0.806e-6f; // [V]
    }
};

template <typename BusSPI, typename rstPin>
constexpr float ADIS16405<BusSPI,rstPin>::gyroFSMAP[];

#endif
