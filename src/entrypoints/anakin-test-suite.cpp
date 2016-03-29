/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Alain Carlucci, Matteo Piazzolla
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
#include <sensors/FXAS21002.h>
#include <sensors/MAX21105.h>
#include <sensors/MPU9250.h>
#include <sensors/MS580301BA07.h>
//#include <sensors/Si7021.h>

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

void banner() {
printf("------------------------------------------------------------------------------\n"
       "d888888b d88888b .d8888. d888888b   .d8888. db    db d888888b d888888b d88888b\n"
       "`~~88~~' 88'     88'  YP `~~88~~'   88'  YP 88    88   `88'   `~~88~~' 88'    \n"
       "   88    88ooooo `8bo.      88      `8bo.   88    88    88       88    88ooooo\n"
       "   88    88~~~~~   `Y8b.    88        `Y8b. 88    88    88       88    88~~~~~\n"
       "   88    88.     db   8D    88      db   8D 88b  d88   .88.      88    88.    \n"
       "   YP    Y88888P `8888Y'    YP      `8888Y' ~Y8888P' Y888888P    YP    Y88888P\n"
       "------------------------------------------------------------------------------\n\n");
}

void test_and_print(const char *name, uint8_t reg, uint8_t test, uint8_t reference) {
    if (test == reference) 
        ++num_ok;
    else
        ++num_fail;
    printf("[%13s] [REG 0x%02x]: 0x%02x -> [%s]\n", name, reg, test, (test == reference)?mOK:mKO);
}

void assert_true(const char *name, const char *func, bool value) {
    if(value)
        ++num_ok;
    else
        ++num_fail;

    printf("[%13s] [FUNC %9s] -> [%s]\n", name, func, (value)?mOK:mKO);
}

void test_max31856() {
    // Check if a bunch of register have the reference value
    TEST(MAX31856,  0x00, 0x00); // config 0
    TEST(MAX31856,  0x01, 0x03); // config 1
    TEST(MAX31856,  0x02, 0xFF); // mask 
    TEST(MAX31856,  0x03, 0x7F); // hi fault
    TEST(MAX31856,  0x04, 0xc0); // lo fault
    TEST(MAX31856,  0x05, 0x7f); // lol
    TEST(MAX31856,  0x0F, 0x00); // fault status reg
}

void test_altimeter() {
	MS580301BA07<spiMS580301BA07> altiMS580;

	// Read and check different const registers
    TEST(MS580301BA07, 0xA4, READ(MS580301BA07, 0xA4));
    TEST(MS580301BA07, 0xAA, READ(MS580301BA07, 0xAA));
    TEST(MS580301BA07, 0xAC, READ(MS580301BA07, 0xAC));
	assert_true("MS580301BA07", "init()", altiMS580.init());
}

int main() {
    MAX21105<spiMAX21105> max21(
        MAX21105<spiMAX21105>::ACC_FS_2G,
        MAX21105<spiMAX21105>::GYRO_FS_500
    );

    banner();
    // Pausa per lasciare il tempo ai sensori di accendersi 
    Thread::sleep(100);

    printf("Polling sensors...\n");

    // First check: compare sensors WHO_AM_I against the reference value
    TEST(MPU9250,   0x75, 0x71);
    TEST(FXAS21002, 0x0c, 0xd7);
    TEST(MAX21105,  0x20, 0xb4);
    TEST(LSM9DS0_G, 0x0f, 0xd4);
    TEST(LSM9DS0_A, 0x0f, 0x49);
    TEST(LPS331AP, 0x0f, 0xBB);

    test_max31856();
    test_altimeter();

    assert_true("MAX21105", "init()", max21.init());

    printf("---------------------\n");
    printf("NUM TEST OK:  %5d\n"
           "NUM TEST FAIL:%5d\n\n", num_ok, num_fail);

#ifdef ACCEL_ENDLESS_TEST
    uint16_t cnt=0;
    while(true) {
        max21.updateParams(); 
        Vec3 a = max21.getAccel();
        Vec3 g = max21.getOrientation();
        float temp = max21.getTemperature();

        printf("%05u[ACC %+5.2f,%+5.2f,%+5.2f] "
               "[GYR %+5.2f,%+5.2f,%+5.2f] "
               "[TMP %+5.2fC] \r",
                (++cnt), a.get(0), a.get(1), a.get(2),
                g.get(0),g.get(1),g.get(2), temp);
        Thread::sleep(10);
    }
#endif

    // Bye.
    while(1) { Thread::sleep(1000); }
}
