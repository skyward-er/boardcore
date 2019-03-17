/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta <luca.erbetta@skywarder.eu>
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
#include <interfaces-impl/hwmapping.h>
#include "boards/Homeone/configs/SensorManagerConfig.h"
#include "sensors/MPU9250/MPU9250.h"

using miosix::Thread;
using namespace HomeoneBoard;

enum REGS
{
    REG_WHO_AM_I     = 117,
    REG_USER_CONTROL = 106
};

uint8_t who_am_i_value = 0x68;

void readWhoAmI() { printf("WHO AM I: %d\n", spiMPU9250::read(REG_WHO_AM_I)); }

typedef MPU9250<spiMPU9250> MPU9250Type;
int main()
{
    spiMAX21105::init();
    spiMPU9250::init();

    MPU9250Type imu{1, 1};

    // suint8;_t user_ctrl = spiMPU9250::read(REG_USER_CONTROL);
    // spiMPU9250::write(REG_USER_CONTROL, 16);

    Thread::sleep(500);

    // MPU9250<spiMPU9250> mpu(1, 1);

    while (true)
    {
        imu.init();
        // readWhoAmI();
        // mpu.init();
        miosix::ledOn();
        // printf("Serial is working!\n");
        Thread::sleep(500);
        miosix::ledOff();
        Thread::sleep(500);
    }
}
