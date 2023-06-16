/* Copyright (c) 2022 Skyward Experimental Rocketry
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

#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <utils/Debug.h>

#include <algorithm>
#include <iostream>  //////////////////////////////////////////////////////////////////////////////////////////////

using namespace Boardcore;
using namespace miosix;

int main()
{

    SPIBus bus(SPI3);

    GpioPin csPin(GPIOE_BASE, 3);  // PE3 CS
    csPin.mode(Mode::OUTPUT);

    GpioPin clockPin(GPIOB_BASE, 3);  // PB3 CK (SCL)
    clockPin.mode(Mode::ALTERNATE);
    clockPin.alternateFunction(6);
    GpioPin misoPin(GPIOB_BASE, 4);  // PB4 MISO (SDO)
    misoPin.alternateFunction(6);
    misoPin.mode(Mode::ALTERNATE);
    GpioPin mosiPin(GPIOB_BASE, 5);  // PB5 MOSI (SDA)
    mosiPin.alternateFunction(6);
    mosiPin.mode(Mode::ALTERNATE);

    GpioPin int1Pin(GPIOC_BASE, 15);  // PC15 interrupt pin 1
    int1Pin.mode(Mode::INPUT);
    GpioPin int2Pin(GPIOC_BASE, 13);  // PC13 interrupt pin 2
    int2Pin.mode(Mode::INPUT);

    SPIBusConfig busConfiguration;  // Bus configuration for the sensor
    busConfiguration.clockDivider = SPI::ClockDivider::DIV_256;
    busConfiguration.mode =
        SPI::Mode::MODE_0;  // Set clock polarity to 0 and phase to 1

    LSM6DSRXConfig sensConfig;
    sensConfig.bdu = LSM6DSRXConfig::BDU::CONTINUOUS_UPDATE;

    // acc
    sensConfig.fsAcc     = LSM6DSRXConfig::ACC_FULLSCALE::G2;
    sensConfig.odrAcc    = LSM6DSRXConfig::ACC_ODR::HZ_12_5;
    sensConfig.opModeAcc = LSM6DSRXConfig::OPERATING_MODE::NORMAL;

    // gyr
    sensConfig.fsGyr     = LSM6DSRXConfig::GYR_FULLSCALE::DPS_125;
    sensConfig.odrGyr    = LSM6DSRXConfig::GYR_ODR::HZ_12_5;
    sensConfig.opModeGyr = LSM6DSRXConfig::OPERATING_MODE::NORMAL;

    // fifo
    sensConfig.fifoMode = LSM6DSRXConfig::FIFO_MODE::CONTINUOUS;
    sensConfig.fifoTimestampDecimation =
        LSM6DSRXConfig::FIFO_TIMESTAMP_DECIMATION::DEC_1;
    sensConfig.fifoTemperatureBdr =
        LSM6DSRXConfig::FIFO_TEMPERATURE_BDR::DISABLED;

    // interrupt
    sensConfig.int1InterruptSelection = LSM6DSRXConfig::INTERRUPT::GYR_DRDY;
    sensConfig.int2InterruptSelection =
        LSM6DSRXConfig::INTERRUPT::FIFO_THRESHOLD;
    sensConfig.fifoWatermark = 100;

    LSM6DSRX sens(bus, csPin, busConfiguration, sensConfig);

    if (sens.init() == false)
    {
        while (true)
        {
            TRACE("Error, sensor not initialized\n\n");
            Thread::sleep(2000);
        }
    }

    if (sens.selfTest())
    {
        TRACE("Self test successful\n\n");
        Thread::sleep(2000);
    }
    else
    {
        TRACE("Self test failed\n\n");
        while (true)
        {
            Thread::sleep(2000);
        }
    }

    // while(true)
    // {
    //     Thread::sleep(5000);
    // }

    // Thread::sleep(
    //     1000);  // sleep in order to produce some data before reading from
    //     FIFO.

    // LSM6DSRX::SensorData data{0.0, 0.0, 0.0};
    LSM6DSRXData data;
    // const int SIZE = 60;
    // LSM6DSRX::FifoData buf[SIZE];

    while (true)
    {
        data = sens.getSensorData();

        std::cout << data.header() << "\n";
        data.print(std::cout);
        std::cout << "\n\n";

        // gyroscope, wait for data ready
        // dataReady = int1Pin.value();
        // while (dataReady != 1)
        // {
        //     Thread::sleep(20);
        //     dataReady = int1Pin.value();
        // }
        // sens.getGyroscopeData(data);
        // if(std::max({data.x, data.y, data.z}) > 1000.0 || std::min({data.x,
        // data.y, data.z}) < -1000.0)
        // {
        //     TRACE("Gyroscope:\n");
        //     TRACE("x: %f\n", data.x);
        //     TRACE("y: %f\n", data.y);
        //     TRACE("z: %f\n\n\n", data.z);
        // }

        // accelerometer, wait for data ready
        // dataReady = int2Pin.value();
        // while (dataReady != 1)
        // {
        //     Thread::sleep(20);
        //     dataReady = int2Pin.value();
        // }
        // sens.getAccelerometerData(data);
        // TRACE("Accelerometer:\n");
        // TRACE("x: %f\n", data.x);
        // TRACE("y: %f\n", data.y);
        // TRACE("z: %f\n\n", data.z);

        // wait for fifo full interrupt
        // int dataReady = int2Pin.value();
        // while (dataReady != 1)
        // {
        //     Thread::sleep(20);
        //     dataReady = int2Pin.value();
        // }

        // TRACE(
        //     "Interrupt ricevuto\n"
        //     "dati non letti pre-lettura: %u\n",
        //     sens.unreadDataInFifo());
        // const int numBatchRed = sens.readFromFifo(buf, SIZE);
        // TRACE("numero dati letti: %d\n", numBatchRed);
        // TRACE("dati non letti post-lettura: %u\n\n",
        // sens.unreadDataInFifo());

        // float sensitivity = 0.0;
        // for (int i = 0; i < numBatchRed && i < 4; ++i)
        // {
        //     // read sensor tag
        //     uint8_t tagSensor = (buf[i].tag >> 3) & 31;
        //     switch (tagSensor)
        //     {
        //         case 0x01:
        //             // gyroscope
        //             TRACE("%d) GYROSCOPE\n", i);
        //             sensitivity = 4.375;
        //             break;
        //         case 0x02:
        //             // accelerometer
        //             TRACE("%d) ACCELEROMETER\n", i);
        //             sensitivity = 0.061;
        //             break;
        //         case 0x04:
        //             // timestamp
        //             TRACE("%d) TIMESTAMP\n", i);
        //             sensitivity = 1.0;
        //             break;
        //         default:
        //             TRACE("%d) UNRECOGNIZED DATA\n", i);
        //             sensitivity = 1.0;
        //             break;
        //     }

        //     if (tagSensor != 0x04)
        //     {
        //         // print data

        //         // note: data from sensors has to be casted as signed, before
        //         // casting again as float.

        //         TRACE("x: %f\n",
        //               static_cast<float>(static_cast<int16_t>(buf[i].x)) *
        //                   sensitivity);
        //         TRACE("y: %f\n",
        //               static_cast<float>(static_cast<int16_t>(buf[i].y)) *
        //                   sensitivity);
        //         TRACE("z: %f\n\n\n",
        //               static_cast<float>(static_cast<int16_t>(buf[i].z)) *
        //                   sensitivity);
        //     }
        //     else
        //     {
        //         // print timestamp data
        //         uint32_t timestamp = static_cast<uint32_t>(buf[i].x);
        //         timestamp |= static_cast<uint32_t>(buf[i].y) << 16;
        //         TRACE("Value: %u\n\n\n", timestamp);
        //     }
        // }

        Thread::sleep(2000);
    }

    return 0;
}

// #include <ctime>
// uint64_t differenza(const uint64_t val1, const uint64_t val2)
// {
//     if(val1 > val2)
//     {
//         return val1 - val2;
//     }

//     return val2 - val1;
// }
// int main()
// {
//     // timestamp test
//     srand(time(NULL));

//     SPIBus bus(SPI3);

//     GpioPin csPin(GPIOE_BASE, 3);  // PE3 CS
//     csPin.mode(Mode::OUTPUT);

//     GpioPin clockPin(GPIOB_BASE, 3);  // PB3 CK (SCL)
//     clockPin.mode(Mode::ALTERNATE);
//     clockPin.alternateFunction(6);
//     GpioPin misoPin(GPIOB_BASE, 4);  // PB4 MISO (SDO)
//     misoPin.alternateFunction(6);
//     misoPin.mode(Mode::ALTERNATE);
//     GpioPin mosiPin(GPIOB_BASE, 5);  // PB5 MOSI (SDA)
//     mosiPin.alternateFunction(6);
//     mosiPin.mode(Mode::ALTERNATE);

//     GpioPin int1Pin(GPIOC_BASE, 15);  // PC15 interrupt pin 1
//     int1Pin.mode(Mode::INPUT);
//     GpioPin int2Pin(GPIOC_BASE, 13);  // PC13 interrupt pin 2
//     int2Pin.mode(Mode::INPUT);

//     SPIBusConfig busConfiguration;  // Bus configuration for the sensor
//     busConfiguration.clockDivider = SPI::ClockDivider::DIV_256;
//     busConfiguration.mode =
//         SPI::Mode::MODE_0;  // Set clock polarity to 0 and phase to 1

//     LSM6DSRXConfig sensConfig;
//     sensConfig.bdu = LSM6DSRXConfig::BDU::CONTINUOUS_UPDATE;

//     // acc
//     sensConfig.fsAcc     = LSM6DSRXConfig::ACC_FULLSCALE::G2;
//     sensConfig.odrAcc    = LSM6DSRXConfig::ACC_ODR::HZ_1_6;
//     sensConfig.opModeAcc = LSM6DSRXConfig::OPERATING_MODE::NORMAL;

//     // gyr
//     sensConfig.fsGyr     = LSM6DSRXConfig::GYR_FULLSCALE::DPS_125;
//     sensConfig.odrGyr    = LSM6DSRXConfig::GYR_ODR::HZ_12_5;
//     sensConfig.opModeGyr = LSM6DSRXConfig::OPERATING_MODE::NORMAL;

//     // fifo
//     sensConfig.fifoMode = LSM6DSRXConfig::FIFO_MODE::BYPASS;
//     sensConfig.fifoTimestampDecimation =
//         LSM6DSRXConfig::FIFO_TIMESTAMP_DECIMATION::DISABLED;
//     sensConfig.fifoTemperatureBdr =
//         LSM6DSRXConfig::FIFO_TEMPERATURE_BDR::DISABLED;

//     // interrupt
//     sensConfig.int1InterruptSelection = LSM6DSRXConfig::INTERRUPT::NOTHING;
//     sensConfig.int2InterruptSelection =
//         LSM6DSRXConfig::INTERRUPT::NOTHING;
//     sensConfig.fifoWatermark = 0;

//     LSM6DSRX sens(bus, csPin, busConfiguration, sensConfig);

//     if (sens.init() == false)
//     {
//         while (true)
//         {
//             TRACE("Error, sensor not initialized\n\n");
//             Thread::sleep(2000);
//         }
//     }

//     TRACE("Inizio test\n\n");

//     const int NUM_TEST = 10;
//     int i = 0;

//     uint64_t legacy0 = 0, legacy1 = 0;// misurazioni a t0 e t1
//     uint64_t legacy1calcolato = 0;// valore legacy a t1 calcolato partendo
//     dai dati del sensore uint64_t sens0 = 0, sens1 = 0;// misurazioni a t0 e
//     t1 uint64_t deltaSens = 0, deltaLegacy = 0;// variazione tra la misura a
//     t0 e quella a t1 uint64_t diff = 0;// differenza tra legacy1 e
//     legacy1calcolato

//     legacy0 = TimestampTimer::getTimestamp();
//     sens0 = sens.getSensorTimestamp();

//     const float sensResolution = sens.getSensorTimestampResolution() *
//     1000;// la funzione restituisce in ms TRACE("Resolution: %f\n",
//     sensResolution); for(i = 0; i < NUM_TEST; ++i)
//     {
//         Thread::sleep((rand() % 2000) + 1000);

//         legacy1 = TimestampTimer::getTimestamp();
//         sens1 = sens.getSensorTimestamp();

//         deltaSens = sens1 - sens0;
//         deltaLegacy = deltaSens * sensResolution;

//         legacy1calcolato = legacy0 + deltaLegacy;

//         diff = differenza(legacy1, legacy1calcolato);

//         TRACE("Campione %d:\n"
//                 "legacy1:\t\t%llu\n"
//                 "legacy1calcolato:\t%llu\n"
//                 "differenza:\t\t%llu\n\n", i, legacy1, legacy1calcolato,
//                 diff);

//         // prepara per i cicli successivi
//         legacy0 = legacy1;
//         sens0 = sens1;
//     }

//     while(true)
//     {
//         Thread::sleep(5000);
//     }

//     return 0;
// }
