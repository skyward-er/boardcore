/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Alain Carlucci, Matteo Michele Piazzolla
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
#include <sensors/MAX21105.h>
#include <sensors/MPU9250.h>
#include <sensors/iNemo.h>

using namespace miosix;

#define SENSOR(NAME,CSPORT,CSPIN) \
    typedef Gpio<GPIO ## CSPORT ## _BASE, CSPIN> CS_ ## NAME; \
    typedef ProtocolSPI < busSPI1, CS_ ## NAME > spi ## NAME

#define READ(NAME, ID) spi ## NAME::read(ID)
#define TEST(NAME, REG, REF) test_and_print(#NAME, REG, READ(NAME, REG), REF)

#define ACCEL_ENDLESS_TEST

// SPI1 
typedef Gpio<GPIOA_BASE, 5> GpioSck;
typedef Gpio<GPIOA_BASE, 6> GpioMiso;
typedef Gpio<GPIOA_BASE, 7> GpioMosi;
typedef BusSPI < 1, GpioMosi, GpioMiso, GpioSck> busSPI1;

// Here's the list of SPI sensors that should be tested
//    | NAME      | CSPORT | CSPIN |
SENSOR(MPU9250,      D,       13);
SENSOR(FXAS21002,    G,       10);
SENSOR(MAX21105,     E,        2);
SENSOR(MAX31856,     B,       11);
SENSOR(LSM9DS0_G,    G,        9);
SENSOR(LSM9DS0_A,    G,       11);
SENSOR(LPS331AP,     E,        4);
SENSOR(MS580301BA07, B,       10);

static const char mOK[] = "OK";
static const char mKO[] = "FAIL";

int num_ok = 0, num_fail = 0;

typedef MAX21105<spiMAX21105> max_t;
typedef iNEMOLSM9DS0<spiLSM9DS0_G, spiLSM9DS0_A> lsm_t;
typedef MPU9250<spiMPU9250> mpu_t;
typedef struct { Vec3 v[3]; } vec_t;

Vec3 calcMid(vec_t data[5], int j) {
    Vec3 ret;

    for(int i=0;i<5;i++)
        ret += Vec3(data[i].v[j]);

    ret *= 1.0/5.0;

    return ret;
}

void calibrate(const vec_t& data[10]) {
    for(const auto& i : data) {
        for(int j=0;j<3;j++) {
            i->v[j].normalize();
            i->v[j] *= EARTH_GRAVITY;
        }
    }


}

int main() {
    vec_t data[5];
    vector<AccelSensor *> accels;
    accels.push_back(new lsm_t(lsm_t::ACC_FS_2G, 
                lsm_t::GYRO_FS_245, lsm_t::COMPASS_FS_2));
    accels.push_back(new mpu_t(mpu_t::ACC_FS_2G, mpu_t::GYRO_FS_250));
    accels.push_back(new max_t(max_t::ACC_FS_2G, max_t::GYRO_FS_250));

    uint16_t cnt=0;

    for(auto jj : accels)
        jj->init();

    printf("Wait..\n");
    Thread::sleep(2000);
    printf("Calibrating..\n");

    vec_t caa[10];

    for(int i=0;i<10;i++) {
        for(int j=0;j<3;j++) {
            AccelSensor *a = accels[j];
            a->updateParams();
            caa[i].v[j] = a->getAccel();
        }
        printf("%02d/10...\r", i+1);
        Thread::sleep(20);
    }

    calibrate(caa);

    while(true) {
        printf("%05d ", cnt);
        for(int j=0;j<3;j++) {
            AccelSensor *i = accels[j];
            i->updateParams();
            data[cnt%5].v[j] = i->getAccel();
            
            Vec3 mid = adjust(data,j);
            printf("[%+5.2f,%+5.2f,%+5.2f] ", 
                    mid.getX(), mid.getY(), mid.getZ());
        }
        printf("     \r");
        cnt++;
        Thread::sleep(10);
    }
}
