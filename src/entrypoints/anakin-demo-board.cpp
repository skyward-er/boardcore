/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
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
#include <sensors/FXAS21002.h>
#include <sensors/LPS331AP.h>
#include <sensors/MAX21105.h>
#include <sensors/MAX31856.h>
#include <sensors/MPL3115.h>
#include <sensors/MPU9250.h>
#include <sensors/MS580301BA07.h>
#include <sensors/Si7021.h>
#include <sensors/iNemo.h>
#include <drivers/stm32f2_f4_i2c.h> 
#include <ethernet/UdpSocket.h>

using namespace miosix;

#define SENSOR(NAME,CSPORT,CSPIN) \
    typedef Gpio<GPIO ## CSPORT ## _BASE, CSPIN> CS_ ## NAME; \
    typedef ProtocolSPI < busSPI1, CS_ ## NAME > spi ## NAME

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

#pragma pack(1)
struct pkt_t {
    float accel[3];
    float gyro[3];
    float compass[3];
    float temperature;
    float humidity;
    float pressure;
};
#pragma pack()

// Please don't judge me for these lines
#define PUSH(a,x) do { sensors.push_back(a); x.push_back(a); } while(0)
#define PUSH2(a,x,y) do { PUSH(a,x); y.push_back(a); } while(0)
#define PUSH3(a,x,y,z) do { PUSH2(a,x,y); z.push_back(a); } while(0)

class DemoBoard : public Singleton<DemoBoard> {
    friend class Singleton<DemoBoard>;
public:
    void init() {}

    void update() {
        for(Sensor *s : sensors)
            s->updateParams();
    }
    #define FUNC(x,y,z,w) x y () const \
        { x j; for(z *i : w){j.push_back(i->y());} return j; }

    FUNC(vector<Vec3>,  getAccel,       AccelSensor,        accel)
    FUNC(vector<Vec3>,  getRotation,    GyroSensor,         gyro)
    FUNC(vector<Vec3>,  getCompass,     CompassSensor,      compass)
    FUNC(vector<float>, getTemperature, TemperatureSensor,  temps)
    FUNC(vector<float>, getHumidity,    HumiditySensor,     humidity)
    FUNC(vector<float>, getPressure,    PressureSensor,     pressure)

#undef FUNC
private:
    DemoBoard() {
        // ----- FXAS21102 -----
        typedef FXAS21002<spiFXAS21002> fx_t; 
        fx_t *fx = new fx_t();
        if(fx->init())
            PUSH(fx, gyro);
        else 
            printf("Cannot start FXAS21102\n");

        // ----- iNEMO -----
        typedef iNEMOLSM9DS0<spiLSM9DS0_G, spiLSM9DS0_A> pd_inemo_t;
        pd_inemo_t *nemo = new pd_inemo_t(pd_inemo_t::ACC_FS_2G, 
                                          pd_inemo_t::GYRO_FS_245,
                                          pd_inemo_t::COMPASS_FS_2);
        if(nemo->init())
            PUSH3(nemo, accel, gyro, compass);
        else
            printf("Cannot start iNemo\n");

        // ----- LPS331AP -----
        typedef LPS331AP<spiLPS331AP> lps_t;
        lps_t *lps = new lps_t(lps_t::SS_25HZ);
        if(lps->init())
            PUSH2(lps, pressure, temps);
        else
            printf("Cannot start LPS\n");

        // ----- MAX21105 -----
        typedef MAX21105<spiMAX21105> max21_t;
        max21_t *max21 = new max21_t(max21_t::ACC_FS_2G, max21_t::GYRO_FS_500);
        if(max21->init())
            PUSH3(max21, accel, gyro, temps);
        else
            printf("Cannot start MAX21105\n");

        // ----- MPL3115 -----
        sensors::sda::mode(Mode::ALTERNATE_OD);
        sensors::sda::alternateFunction(4);
        sensors::scl::mode(Mode::ALTERNATE_OD);
        sensors::scl::alternateFunction(4);
 
        typedef MPL3115< ProtocolI2C<miosix::I2C1Driver> > mpl_t;
        mpl_t *mpl = new mpl_t();
        if(mpl->init())
            PUSH2(mpl, pressure, temps);
        else
            printf("Cannot start MPL3115\n");
        
        // ----- MPU9250 -----
        typedef MPU9250<spiMPU9250> mpu_t;
        mpu_t *mpu = new mpu_t(mpu_t::ACC_FS_2G, mpu_t::GYRO_FS_250);
        if(mpu->init())
            PUSH3(mpu, accel, compass, temps);
        else
            printf("Cannot start MPU9250\n");

        // ----- MS580301BA07 -----
        typedef MS580301BA07<spiMS580301BA07> ms5_t;
        ms5_t *ms5 = new ms5_t();
        if(ms5->init())
            PUSH2(ms5, pressure, temps);
        else
            printf("Cannot start MS580\n");

        // ----- Si7021 -----
        typedef Si7021< ProtocolI2C<miosix::I2C1Driver> > si_t;
        si_t *si = new si_t();
        if(si->init())
            PUSH2(si, humidity, temps);
        else
            printf("Cannot start Si7021\n");

        printf("Loaded %d sensors\n", sensors.size());
        printf("  + %d gyro sensors\n", gyro.size());
        printf("  + %d accel sensors\n", accel.size());
        printf("  + %d compass sensors\n", compass.size());
        printf("  + %d temperature sensors\n", temps.size());
        printf("  + %d humidity sensors\n", humidity.size());
        printf("  + %d pressure sensors\n", pressure.size());
    }

    vector<Sensor *>            sensors;
    vector<GyroSensor *>        gyro;
    vector<AccelSensor *>       accel;
    vector<CompassSensor *>     compass;
    vector<TemperatureSensor *> temps;
    vector<HumiditySensor *>    humidity;
    vector<PressureSensor *>    pressure;
    #define sDemoBoard DemoBoard::getInstance()
};

Vec3 averageVec(const vector<Vec3>& in) {
    Vec3 out;
    for(const Vec3& i : in)
        out += i;

    if(in.size() > 0)
        out *= 1.0f / (float)in.size();
    return out;
}

float averageFloat(const vector<float>& in) {
    float out = 0;
    for(const float& i : in)
        out += i;
    if(in.size() > 0)
        out *= 1.0f / (float)in.size();
    return out;
}

int main() {
    Thread::sleep(100);
    
    //Ethernet setup
    const uint8_t ipAddr[] = {192,168,1,30}; // Device's IP address
    const uint8_t mask[] = {255,255,255,0};  // Subnet mask
    
    const uint8_t destIp[] = {192,168,1,4};  // Destination IP address
    const uint16_t destPort = 1234;          // Destination port
            
    W5200& eth = W5200::instance();
    eth.setIpAddress(ipAddr);
    eth.setSubnetMask(mask);
    
    //This socket listens on UDP port 2020
    UdpSocket sock(2020);    

    sDemoBoard->init();
    while(1) {
        sDemoBoard->update();
        Thread::sleep(20);

        vector<Vec3> accels = sDemoBoard->getAccel();
        vector<Vec3> rots =   sDemoBoard->getRotation();
        vector<Vec3> compsv = sDemoBoard->getCompass();
        vector<float> temps = sDemoBoard->getTemperature();
        vector<float> humid = sDemoBoard->getHumidity();
        vector<float> press = sDemoBoard->getPressure();

        Vec3 acc = averageVec(accels);
        Vec3 rot = averageVec(rots);
        Vec3 comps = averageVec(compsv);
        float tempm = averageFloat(temps);
        float humim = averageFloat(humid);
        float presm = averageFloat(press);

#define COPYVEC(arr,v) do { \
    arr[0] = v.getX(); arr[1] = v.getY(); arr[2] = v.getZ(); } while(0)

        pkt_t packet;
        COPYVEC(packet.accel, acc);
        COPYVEC(packet.gyro, rot);
        COPYVEC(packet.compass, comps);
        packet.temperature = tempm;
        packet.humidity = humim;
        packet.pressure = presm;

        sock.sendTo(destIp,destPort,&packet,sizeof(packet));

        printf( "[%5.2f %5.2f %5.2f] "
                "[%5.2f %5.2f %5.2f] "
                "[%5.2f %5.2f %5.2f] " 
                "[%5.2f %5.2f %5.2f] \r",
                acc.getX(), acc.getY(), acc.getZ(),
                rot.getX(), rot.getY(), rot.getZ(),
                comps.getX(), comps.getY(), comps.getZ(),
                tempm, humim, presm);

        fflush(stdout);
    }

    // Bye.
    while(1) { Thread::sleep(1000); }
}
