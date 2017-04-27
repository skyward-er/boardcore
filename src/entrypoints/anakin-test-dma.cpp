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
#include <Leds.h>
#include <boards/AnakinBoard.h>

using namespace miosix;


int main()
{
    sBoard->init();

    const std::vector<SingleSensor>& data = sBoard->debugGetSensors();
    int ctr=0;
    while(1)
    {
        printf("---------%05d----------\n", ctr++);
        for(const auto& s : data)
        {
            printf("Sensor %03d:", s.sensor);
            if(s.value == nullptr)
            {
                printf("NULLPTR\n");
                continue;
            }

            switch(s.data)
            {
                case DATA_VEC3:
                {
                    const Vec3* d = (const Vec3*) s.value;
                    printf("(%f,%f,%f)\n", d->getX(), d->getY(), d->getZ());
                    break;
                }
                case DATA_FLOAT:
                {
                    const float* d = (const float*) s.value;
                    printf("%f\n", *d);
                    break;
                }
                case DATA_INT:
                {
                    const int* d = (const int*) s.value; 
                    printf("[%08x]\n", *d);
                    break;
                }
                default:
                {
                    printf("Unhandled %d\n", s.data);
                    break;
                }
            }
        }
        printf("-----------------------\n");
        Thread::sleep(10);
    }
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
