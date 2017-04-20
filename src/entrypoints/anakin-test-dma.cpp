/* Copyright (c) 2015-2017 Skyward Experimental Rocketry
 * Authors: Alain Carlucci
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

#include <Common.h>
#include <BusTemplate.h>
#include <sensors/MPU9250.h>
#include <sensors/iNemo.h>
#include <DMA/DMA.h>
#include <Leds.h>
#include <DMA/SensorSampling.h>
#include <events/Scheduler.h>

using namespace miosix;

typedef Gpio<GPIOA_BASE, 5> GpioSck;
typedef Gpio<GPIOA_BASE, 6> GpioMiso;
typedef Gpio<GPIOA_BASE, 7> GpioMosi;
typedef BusSPI < 1, GpioMosi, GpioMiso, GpioSck> busSPI1;

typedef Gpio<GPIOD_BASE, 13> CS_MPU9250;
typedef Gpio<GPIOG_BASE, 9> CS_INEMO_G;
typedef Gpio<GPIOG_BASE, 11> CS_INEMO_A;

typedef ProtocolSPI<busSPI1, CS_MPU9250> spiMPU9250;
typedef ProtocolSPI<busSPI1, CS_INEMO_A> spiINEMOA;
typedef ProtocolSPI<busSPI1, CS_INEMO_G> spiINEMOG;

typedef MPU9250<spiMPU9250> mpu_t;
typedef iNEMOLSM9DS0<spiINEMOG,spiINEMOA> inemo_t;

int main()
{
    SensorSampling ss;

    mpu_t test( mpu_t::ACC_FS_2G, mpu_t::GYRO_FS_250);
    test.init();

    inemo_t inemo(inemo_t::ACC_FS_2G, inemo_t::GYRO_FS_245, inemo_t::COMPASS_FS_2);
    inemo.init();

    ss.AddSensor(dynamic_cast<Sensor*>(&test));
    ss.AddSensor(dynamic_cast<Sensor*>(&inemo));

    sEventScheduler->add(std::bind(&SensorSampling::Update, ss), 10, "Sample 10ms");
    /*
    std::vector<SPIRequest> requests;
    requests.push_back(
        SPIRequest(CS_MPU9250::getPin(),{0x80 | 0x75, 0, 0, 0, 0, 0, 0, 0, 0})
    );

    printf("A\n");
    auto& driver=SPIDriver::instance();
    printf("B\n");
    bool ret = driver.transaction(requests);
    printf("C: %d\n", ret);
    auto result=requests[0].readResponseFromPeripheral();
    printf("D\n");
    memDump(result.data(),result.size());
    printf("E\n");
    */

    while(1){
        // Yo
    }
    return 0;
}
