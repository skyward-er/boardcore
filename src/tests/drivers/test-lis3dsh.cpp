/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Luca Conterio
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

#include <miosix.h>
#include <drivers/spi/SPIDriver.h>
#include "sensors/LIS3DSH/LIS3DSH.h"
#include "sensors/LIS3DSH/LIS3DSHData.h"

using namespace std;
using namespace miosix;

SPIBus bus(SPI1);
GpioPin cs(GPIOE_BASE, 3);

int main() {

    {
        FastInterruptDisableLock dLock;

        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // SPI1 ENABLE

        cs.mode(Mode::OUTPUT);
    }
    cs.high();
    
    LIS3DSH sensor(
            bus,
            cs,
            sensor.ODR_100_HZ, 
            sensor.UPDATE_AFTER_READ_MODE, 
            sensor.FULL_SCALE_4G
    );

    Vec3 acc;
    int8_t temp;

    // sensor not initialized, should return false
    if (sensor.onSimpleUpdate() || sensor.updateTemperature()) {
        printf("\nTest failed: sensor not initialized \n");
        return -1;
    }

    bool initialized = false;
    // initialize imu
    if (!sensor.init()) {
        if (sensor.getLastError() == sensor.ERR_NOT_ME) {
            printf("Test failed: invalid WHO_AM_I value, init failed \n");
        }
        else {
            printf("Test failed: init failed \n");
        }
        return -1;
    }
    initialized = true;

    // check if the sensor is properly working
    if (!sensor.selfTest()) {
        printf("\nTest failed: self-test failed \n");
        return -1;
    }

    // if sensor already inizialized, init() should return false
    if (initialized) {
        if (sensor.init()) {
            printf("\nTest failed: sensor is already initialized \n");
            return -1;
        }
    }

    Thread::sleep(500);
    
    // sample some data from the sensor
    for(int i = 0; i < 5; i++) {
        sensor.updateTemperature();
        // sensor intitialized, should return true (false if no new data exist)
        if (!sensor.onSimpleUpdate()) {
            printf("\nWarning: no new data to be read \n");
        }

        acc = *(sensor.accelDataPtr());
        temp = (int8_t) *(sensor.tempDataPtr());

        printf("\nAccel: x: %.2f | y: %.2f | z: %.2f \n", acc.getX(), acc.getY(), acc.getZ());
        printf("Temp: %d C \n", temp);
        
        Thread::sleep(200);
    }

    printf("\nTest ok \n");

    return 0;
}