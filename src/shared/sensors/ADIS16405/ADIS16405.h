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

// TODO: add ADC/DAC interface (left floating on the schematics but ask)
// TODO: add calibration routines
// TODO: add sleep/wakeup routines
// TODO: add power management routines
// TODO: add global commands routines
// TODO: add sample rate routines (although it's strictly
//       not recommended to change the e default sample rate)
// TODO: add digital filtering routines (ask if needed)
// TODO: add dynamic range  routines
// TODO: add GPIO interface (ask if used)
// TODO: add memory test routine


// 16g,250hz
template <typename BusSPI, typename rstPin>
class ADIS16405 : public AccelSensor,
                  public GyroSensor,
                  public CompassSensor,
                  public TemperatureSensor
{
public:
    ADIS16405() {}
    virtual ~ADIS16405()
    {
        //TODO: should the device be put to sleep?
    }

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
        rstPin::high();

        miosix::delayMs(220); // 220 ms start-up time 

        // TODO: wakeUp the device if it's sleeping
        
        uint16_t product_id = readReg(ADIS_PRODUCT_ID);
        if (product_id != product_id_value)
        {
            last_error = ERR_NOT_ME;
            return false;
        }

        writeReg(ADIS_ALM_CTRL,0x0000); // disable alarms

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

        // writeReg(ADIS_ZMAGN_SIF,0x0800);
        // miosix::delayMs(1);
        // uint16_t reg_value = readReg(ADIS_ZMAGN_SIF);
        // printf("reg value: %d\n", reg_value);
        // miosix::delayMs(200);
    }

    void burstTest()
    {
        ADIS16405Data burstData;
        burstDataCollect(&burstData);
        printf("%f\t",normalizeSupply(burstData.supply_out));
        // printf("%f\t",normalizeGyro(burstData.xgyro_out));
        // printf("%f\t",normalizeGyro(burstData.ygyro_out));
        // printf("%f\t",normalizeGyro(burstData.zgyro_out));
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

        // uint16_t msc = readReg(ADIS_MSC_CTRL);
        // writeReg(ADIS_MSC_CTRL, msc | 1 << 10);
        // msc = readReg(ADIS_MSC_CTRL);
        
        writeReg(ADIS_MSC_CTRL, 0x3504); // Config and start self test
        uint16_t msc = 0;
        // MSC_CTRL[10] resets itself to 0 after completing the routine
        do
        {
            msc = readReg(ADIS_MSC_CTRL);
        } while (msc & 1 << 10);

        // TODO: break the test into all the cases (identify the the fail bit)
        diagstat = readReg(ADIS_DIAG_STAT);  // 0 if successful
        printf("diagstat: %d\n",diagstat);
        return (diagstat == 0);
    }

    void onDMAUpdate(const SPIRequest& req) override
    {
        // TODO: remove. Added to fix warning
        (void) req;

        // const std::vector<uint8_t>& r = req.readResponseFromPeripheral();
        // uint8_t rxbuf_data[sizeof(ADIS16405Data)+2];

        // // initializing array 
        // for (uint16_t i = 0; i < sizeof(ADIS16405Data) + 2; i++){ 
        //     rxbuf_data[i] = 0; 
        // } 

        // memcpy(&rxbuf_data, &(r[2]),
        //        sizeof(rxbuf_data));  // coping from 2nd, the first 2 are address

        // ADIS16405Data* data = NULL;
        // bufferToBurstData(rxbuf_data + 2, data);  // first 2 bytes are padding

        // mLastGyro.setX(data->xgyro_out);
        // mLastGyro.setY(data->ygyro_out);
        // mLastGyro.setZ(data->zgyro_out);

        // mLastAccel.setX(data->xaccl_out);
        // mLastAccel.setY(data->yaccl_out);
        // mLastAccel.setZ(data->zaccl_out);

        // mLastGyro.setX(data->xmagn_out);
        // mLastGyro.setY(data->ymagn_out);
        // mLastGyro.setZ(data->zmagn_out);

        // mLastTemp = data->temp_out;

        // TODO:
        //  Power supply measurement
        //  Auxiliary ADC measurement
    }

    bool onSimpleUpdate() override { return true; }

private:
    // constexpr static uint16_t product_id_value = 0x4105; // Typo in the datasheet
    constexpr static uint16_t product_id_value = 16405; // == 0x4015

    /* ADIS Register map */
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
        ADIS_PRODUCT_ID = 0x56  //  0x4105   Product identifier
        // clang-format on
    };

    uint16_t readReg(adis_regaddr addr)
    {
        uint8_t rxbuf[2];
        BusSPI::read_adis(addr, rxbuf, 2);
        return rxbuf[0] << 8 | rxbuf[1];
    }

    void writeReg(adis_regaddr addr, uint16_t value)
    {
        BusSPI::write((addr+1) | 0x80, (uint8_t)(value >> 8));
        BusSPI::write(addr | 0x80, (uint8_t) value);
    }

    void burstDataCollect(ADIS16405Data* data)
    {   
        uint8_t rxbuf[sizeof(ADIS16405Data)];
        BusSPI::read_adis((ADIS_GLOB_CMD), rxbuf, sizeof(ADIS16405Data));

        // TODO: check nd and ea bits
        data->supply_out = (rxbuf[0] << 8 | rxbuf[1]) & 0x3FFF;
        data->xgyro_out  = signExtend((rxbuf[2] << 8 | rxbuf[3]) & 0x3FFF, 14);
        data->ygyro_out  = signExtend((rxbuf[4] << 8 | rxbuf[5]) & 0x3FFF, 14);
        data->zgyro_out  = signExtend((rxbuf[6] << 8 | rxbuf[7]) & 0x3FFF, 14);
        data->xaccl_out  = signExtend((rxbuf[8] << 8 | rxbuf[9]) & 0x3FFF, 14);
        data->yaccl_out  = signExtend((rxbuf[10] << 8 | rxbuf[11]) & 0x3FFF, 14);
        data->zaccl_out  = signExtend((rxbuf[12] << 8 | rxbuf[13]) & 0x3FFF, 14);
        data->xmagn_out  = signExtend((rxbuf[14] << 8 | rxbuf[15]) & 0x3FFF, 14);
        data->ymagn_out  = signExtend((rxbuf[16] << 8 | rxbuf[17]) & 0x3FFF, 14);
        data->zmagn_out  = signExtend((rxbuf[18] << 8 | rxbuf[19]) & 0x3FFF, 14);
        data->temp_out   = signExtend((rxbuf[20] << 8 | rxbuf[21]) & 0x0FFF, 12);
        data->aux_adc    = (rxbuf[22] << 8 | rxbuf[23]) & 0x0FFF;
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
        return static_cast<float>(val) * 2.418e-3f; // TODO: leave it in V or mV?
    }

    inline float normalizeAccel(int16_t val)
    {
        return static_cast<float>(val) * 3.33e-3f * EARTH_GRAVITY;
    }

    inline float normalizeGyro(int16_t val)
    {
        return static_cast<float>(val) * 0.05f * DEGREES_TO_RADIANS;
    }

    inline float normalizeMagneto(int16_t val)
    {
        return static_cast<float>(val) * 0.5e-3f; // TODO: leave it in gauss?
    }

    inline float normalizeTemp(int16_t val)
    {
        return static_cast<float>((val) * 0.14f + 25);
    }

    inline float normalizeADC(uint16_t val)
    {
        return static_cast<float>(val) * 0.806e-6f; // TODO: leave it in V or mV or uV?
    }

    // TODO
    void sleep(){}
    void sleep(uint8_t dur){}
    void wakeUp(){}
};

#endif
