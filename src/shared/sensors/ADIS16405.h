/* ADIS16405 Driver
 *
 * Copyright (c) 2018 Skyward Experimental Rocketry
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
#include "Sensor.h"

// TODO: add ADC/DAC interface
template <class Bus>
class ADIS16405  : public AccelSensor,
                public GyroSensor,
                public CompassSensor,
                public TemperatureSensor
{
#pragma pack(1)
	/*! \typedef
	 * Burst data collection. This establishes what we consider the right datatype
	 * for the registers because trying to work with 12 or 14 bit twos complement
	 * that doesn't sign extend to 16 bits is unpleasant.
	 */
	typedef struct ADIS16405Data {
		uint16_t supply_out;//  Power supply measurement
		int16_t xgyro_out;  //  X-axis gyroscope output
		int16_t ygyro_out;  //  Y-axis gyroscope output
		int16_t zgyro_out;  //  Z-axis gyroscope output
		int16_t xaccl_out;  //  X-axis accelerometer output
		int16_t yaccl_out;  //  Y-axis accelerometer output
		int16_t zaccl_out;  //  Z-axis accelerometer output
		int16_t xmagn_out;  //  X-axis magnetometer measurement
		int16_t ymagn_out;  //  Y-axis magnetometer measurement
		int16_t zmagn_out;  //  Z-axis magnetometer measurement
		int16_t temp_out;   //  Temperature output
		uint16_t aux_adc;   //  Auxiliary ADC measurement
	} adisdata_t;

#pragma pack()

public:
    ADIS16405() {}
    ~ADIS16405() {}

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

        return {SPIRequest(0, Bus::getCSPin(), v)};
    }

    bool init() override
    {
    	// The module starts itself
    	// Ensure right SPI frequency
    	// Burst mode SPI < 1 MHz
        return true;
    }

    bool selfTest() override
    {
	    // DIAG_STAT clears after each read so we read to clear it
	    uint16_t diagstat = readReg(ADIS_DIAG_STAT);

	    uint16_t msc = readReg(ADIS_MSC_CTRL);
	    writeReg(ADIS_MSC_CTRL, msc | 1<<10);

	    do {
	        msc = readReg(ADIS_MSC_CTRL);
	    } while (msc & 1<<10);

	    diagstat = readReg(ADIS_DIAG_STAT);// 0 if successful, 1 if failed

	    if(diagstat==0)
	    	return true;
	    else
	    	return false; 
    }

    void onDMAUpdate(const SPIRequest &req) override
    {
        const std::vector<uint8_t> &r = req.readResponseFromPeripheral();
        uint8_t raw_data[sizeof(adisdata_t)];

        memcpy(&raw_data, &(r[2]), sizeof(raw_data)); //coping from 2nd, the first 2 are address 

        adisdata_t* data;
        bufferToBurstData(raw_data + 2, data); //first 2 bytes are padding

        mLastGyro.setX(data->xgyro_out);//TODO: do I have to normalize?????
        mLastGyro.setY(data->ygyro_out);//TODO: do I have to normalize?????
        mLastGyro.setZ(data->zgyro_out);//TODO: do I have to normalize?????
        
        mLastAccel.setX(data->xaccl_out);//TODO: do I have to normalize?????
        mLastAccel.setY(data->yaccl_out);//TODO: do I have to normalize?????
        mLastAccel.setZ(data->zaccl_out);//TODO: do I have to normalize?????

        mLastGyro.setX(data->xmagn_out);//TODO: do I have to normalize?????
        mLastGyro.setY(data->ymagn_out);//TODO: do I have to normalize?????
        mLastGyro.setZ(data->zmagn_out);//TODO: do I have to normalize?????

        mLastTemp = data->temp_out;//TODO: do I have to normalize?????

        //TODO:
        //  Power supply measurement
		//  Auxiliary ADC measurement
    }

    bool onSimpleUpdate() override
    {
    	return	true;
    }

private:

	/* ADIS Register addresses */
	enum adis_regaddr
	{
		// Name         address         default    function
		ADIS_FLASH_CNT    = 0x00,        //  N/A     Flash memory write count
		ADIS_SUPPLY_OUT   = 0x02,        //  N/A     Power supply measurement
		ADIS_XGYRO_OUT    = 0x04,        //  N/A     X-axis gyroscope output
		ADIS_YGYRO_OUT    = 0x06,        //  N/A     Y-axis gyroscope output
		ADIS_ZGYRO_OUT    = 0x08,        //  N/A     Z-axis gyroscope output
		ADIS_XACCL_OUT    = 0x0A,        //  N/A     X-axis accelerometer output
		ADIS_YACCL_OUT    = 0x0C,        //  N/A     Y-axis accelerometer output
		ADIS_ZACCL_OUT    = 0x0E,        //  N/A     Z-axis accelerometer output
		ADIS_XMAGN_OUT    = 0x10,        //  N/A     X-axis magnetometer measurement
		ADIS_YMAGN_OUT    = 0x12,        //  N/A     Y-axis magnetometer measurement
		ADIS_ZMAGN_OUT    = 0x14,        //  N/A     Z-axis magnetometer measurement
		ADIS_TEMP_OUT     = 0x16,        //  N/A     Temperature output
		ADIS_AUX_ADC      = 0x18,        //  N/A     Auxiliary ADC measurement
		ADIS_XGYRO_OFF    = 0x1A,        //  0x0000  X-axis gyroscope bias offset factor
		ADIS_YGYRO_OFF    = 0x1C,        //  0x0000  Y-axis gyroscope bias offset factor
		ADIS_ZGYRO_OFF    = 0x1E,        //  0x0000  Z-axis gyroscope bias offset factor
		ADIS_XACCL_OFF    = 0x20,        //  0x0000  X-axis acceleration bias offset factor
		ADIS_YACCL_OFF    = 0x22,        //  0x0000  Y-axis acceleration bias offset factor
		ADIS_ZACCL_OFF    = 0x24,        //  0x0000  Z-axis acceleration bias offset factor
		ADIS_XMAGN_HIF    = 0x26,        //  0x0000  X-axis magnetometer, hard-iron factor
		ADIS_YMAGN_HIF    = 0x28,        //  0x0000  Y-axis magnetometer, hard-iron factor
		ADIS_ZMAGN_HIF    = 0x2A,        //  0x0000  Z-axis magnetometer, hard-iron factor
		ADIS_XMAGN_SIF    = 0x2C,        //  0x0800  X-axis magnetometer, soft-iron factor
		ADIS_YMAGN_SIF    = 0x2E,        //  0x0800  Y-axis magnetometer, soft-iron factor
		ADIS_ZMAGN_SIF    = 0x30,        //  0x0800  Z-axis magnetometer, soft-iron factor
		ADIS_GPIO_CTRL    = 0x32,        //  0x0000  Auxiliary digital input/output control
		ADIS_MSC_CTRL     = 0x34,        //  0x0006  Miscellaneous control
		ADIS_SMPL_PRD     = 0x36,        //  0x0001  Internal sample period (rate) control
		ADIS_SENS_AVG     = 0x38,        //  0x0402  Dynamic range and digital filter control
		ADIS_SLP_CNT      = 0x3A,        //  0x0000  Sleep mode control
		ADIS_DIAG_STAT    = 0x3C,        //  0x0000  System status
		ADIS_GLOB_CMD     = 0x3E,        //  0x0000  System command
		ADIS_ALM_MAG1     = 0x40,        //  0x0000  Alarm 1 amplitude threshold
		ADIS_ALM_MAG2     = 0x42,        //  0x0000  Alarm spi_master_xact_data* caller, spi_master_xact_data* spi_xact, void* data2 amplitude threshold
		ADIS_ALM_SMPL1    = 0x44,        //  0x0000  Alarm 1 sample size
		ADIS_ALM_SMPL2    = 0x46,        //  0x0000  Alarm 2 sample size
		ADIS_ALM_CTRL     = 0x48,        //  0x0000  Alarm control
		ADIS_AUX_DAC      = 0x4A,        //  0x0000  Auxiliary DAC data
		//                = 0x4C to 0x55 //          Reserved
		ADIS_PRODUCT_ID   = 0x56         //          Product identifier
	};

	uint16_t readReg(adis_regaddr addr){
	    uint8_t txbuf[2] = {addr, 0};
	    uint8_t rxbuf[2];

	    Bus::write(txbuf, sizeof(txbuf));
	    Bus::read(rxbuf, sizeof(rxbuf));

	    return rxbuf[0] << 8 | rxbuf[1];
	}

	void writeReg(adis_regaddr addr, uint16_t value){ //todo:array
	    uint8_t txbuf[4] = {
	        ((addr+1) | 0x80), value >> 8,
	        (addr | 0x80),   value
	    };

	    Bus::write(txbuf, sizeof(txbuf));

	    //TODO: check! Do I have to read here??
	    // Bus::read(NULL,sizeof(txbuf));
	}

	int16_t signExtend(uint16_t val, int bits) {
	    if((val&(1<<(bits-1))) != 0){
	        val = val - (1<<bits);
	    }
	    return val;
	}

	void bufferToBurstData(uint8_t * raw, adisdata_t* data){
	    //TODO: check nd and ea bits
	    data->supply_out = (raw[0] << 8 | raw[1]) & 0x3fff;
	    data->xgyro_out  = signExtend((raw[2]   << 8 | raw[3]) & 0x3fff, 14);
	    data->ygyro_out  = signExtend((raw[4]   << 8 | raw[5]) & 0x3fff, 14);
	    data->zgyro_out  = signExtend((raw[6]   << 8 | raw[7]) & 0x3fff, 14);
	    data->xaccl_out  = signExtend((raw[8]   << 8 | raw[9]) & 0x3fff, 14);
	    data->yaccl_out  = signExtend((raw[10] << 8 | raw[11]) & 0x3fff, 14);
	    data->zaccl_out  = signExtend((raw[12] << 8 | raw[13]) & 0x3fff, 14);
	    data->xmagn_out  = signExtend((raw[14] << 8 | raw[15]) & 0x3fff, 14);
	    data->ymagn_out  = signExtend((raw[16] << 8 | raw[17]) & 0x3fff, 14);
	    data->zmagn_out  = signExtend((raw[18] << 8 | raw[19]) & 0x3fff, 14);
	    data->temp_out   = signExtend((raw[20] << 8 | raw[21]) & 0x0fff, 12);
	    data->aux_adc    = (raw[22] << 8 | raw[23]) & 0x0fff;
	}
};

#endif