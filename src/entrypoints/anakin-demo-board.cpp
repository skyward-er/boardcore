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
#include <Leds.h>
#include <sensors/FXAS21002.h>
#include <sensors/LPS331AP.h>
#include <sensors/MAX21105.h>
#include <sensors/MAX31856.h>
#include <sensors/MPL3115.h>
#include <sensors/MPU9250.h>
#include <sensors/MS580301BA07.h>
#include <sensors/Si7021.h>
#include <sensors/iNemo.h>
#include <math/Vec3.h>
#include <math/Matrix.h>
#include <drivers/stm32f2_f4_i2c.h> 
#include <ethernet/UdpManager.h>

using namespace miosix;

//#define ENABLE_ETHERNET
constexpr uint32_t lp_alpha = 250;
constexpr uint32_t hp_beta = 700;

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
    float orientation[3];
};
#pragma pack()

// Please don't judge me for these lines
#define PUSH(a,b,x) do { sensor_t zz = a,b; \
    sensors.push_back(zz); x.push_back(zz); } while(0)
#define PUSH2(a,b,x,y) do { sensor_t zz = a,b; \
    PUSH(a,b,x); y.push_back(zz); } while(0)
#define PUSH3(a,b,x,y,z) do { sensor_t zz = a,b; \
    PUSH2(a,b,x,y); z.push_back(zz); } while(0)
#define PP(name, func) do { printf("[Sensor] " name " started\n");func;}while(0)

class DemoBoard : public Singleton<DemoBoard> {
    friend class Singleton<DemoBoard>;
public:
    void init() {}

    void update() {
        for(const sensor_t& s : sensors)
            (s.sensor)->updateParams();
    }

// Read a float value 
#define RFLO(x,y,k,z,w) vector<x> y () const {               \
    vector<x> j;                                             \
    for(const sensor_t& i : w) {                             \
        j.push_back(*((dynamic_cast<z *>(i.sensor))->k()));  \
    }                                                        \
    return j;                                                \
}

// Read a Vec3 value and multiply it by the sensor transform matrix
#define RVEC(x,y,k,z,w) vector<x> y () const {                             \
    vector<x> j;                                                           \
    for(const sensor_t& i : w) {                                           \
        x vec = *(i.transform) * *((dynamic_cast<z *>(i.sensor))->k());    \
        j.push_back(vec);                                                  \
    }                                                                      \
    return j;                                                              \
}

    RVEC(Vec3,  getAccel,       accelDataPtr,   AccelSensor,        accel)
    RVEC(Vec3,  getRotation,    gyroDataPtr,    GyroSensor,         gyro)
    RVEC(Vec3,  getCompass,     compassDataPtr, CompassSensor,      compass)
    RFLO(float, getTemperature, tempDataPtr,    TemperatureSensor,  temps)
    RFLO(float, getHumidity,    humidityDataPtr,HumiditySensor,     humidity)
    RFLO(float, getPressure,    pressureDataPtr,PressureSensor,     pressure)

#undef RFLO
#undef RVEC
private:
    DemoBoard() {
        static const float v_inemo[16] = { 0,1,0,0, -1,0,0,0, 0,0,1,0, 0,0,0,1};
        static const float v_mpu  [16] = {-1,0,0,0, 0,-1,0,0, 0,0,1,0, 0,0,0,1};
        static const Mat4 idMat = Mat4();
        static const Mat4 iNemoMat = Mat4(v_inemo);
        static const Mat4 mpuMat = Mat4(v_mpu);

        // ----- FXAS21102 -----
        typedef FXAS21002<spiFXAS21002> fx_t; 
        fx_t *fx = new fx_t();
        if(fx->init())
            PP("FXAS21102", PUSH({fx, &idMat}, gyro));
        else 
            printf("Cannot start FXAS21102\n");

        // ----- iNEMO -----
        typedef iNEMOLSM9DS0<spiLSM9DS0_G, spiLSM9DS0_A> pd_inemo_t;
        pd_inemo_t *nemo = new pd_inemo_t(pd_inemo_t::ACC_FS_2G, 
                                          pd_inemo_t::GYRO_FS_245,
                                          pd_inemo_t::COMPASS_FS_2);
        if(nemo->init())
            PP("iNemo",PUSH3({nemo, &iNemoMat}, accel, gyro, compass));
        else
            printf("Cannot start iNemo\n");

        // ----- LPS331AP -----
        typedef LPS331AP<spiLPS331AP> lps_t;
        lps_t *lps = new lps_t(lps_t::SS_25HZ);
        if(lps->init())
            PP("LPS331AP",PUSH2({lps, &idMat}, pressure, temps));
        else
            printf("Cannot start LPS\n");

        // ----- MAX21105 -----
        typedef MAX21105<spiMAX21105> max21_t;
        max21_t *max21 = new max21_t(max21_t::ACC_FS_2G, max21_t::GYRO_FS_500);
        if(max21->init())
            PP("MAX21105",PUSH3({max21, &idMat}, accel, gyro, temps));
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
            PP("MPL3115",PUSH2({mpl, &idMat}, pressure, temps));
        else
            printf("Cannot start MPL3115\n");
        
        // ----- MPU9250 -----
        typedef MPU9250<spiMPU9250> mpu_t;
        mpu_t *mpu = new mpu_t(mpu_t::ACC_FS_2G, mpu_t::GYRO_FS_250);
        if(mpu->init())
            PP("MPU9250",PUSH3({mpu, &mpuMat}, accel, compass, temps));
        else
            printf("Cannot start MPU9250\n");

        // ----- MS580301BA07 -----
        typedef MS580301BA07<spiMS580301BA07> ms5_t;
        ms5_t *ms5 = new ms5_t();
        if(ms5->init())
            PP("MS580301BA07",PUSH2({ms5, &idMat}, pressure, temps));
        else
            printf("Cannot start MS580\n");

        // ----- Si7021 -----
        typedef Si7021< ProtocolI2C<miosix::I2C1Driver> > si_t;
        si_t *si = new si_t();
        if(si->init())
            PP("Si7021",PUSH2({si, &idMat}, humidity, temps));
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

    typedef struct {
        Sensor *sensor;
        const Mat4 *transform;
    } sensor_t;

    vector<sensor_t> sensors;
    vector<sensor_t> accel;
    vector<sensor_t> gyro;
    vector<sensor_t> compass;
    vector<sensor_t> temps;
    vector<sensor_t> humidity;
    vector<sensor_t> pressure;
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

template<uint32_t alpha>
class LowPassPolicy {
protected:
    static float process(uint16_t id, float in) {
        if(id >= storage.size())
            storage.resize(id+1, 0.0);

        storage[id] += (alpha / 1000.0f) * (in - storage[id]);
        return storage[id];
    }
    static vector<float> storage;
};

template<uint32_t beta>
class HighPassPolicy {
protected:
    static float process(uint16_t id, float in) {
        if(id >= storage.size()) {
            storage.resize(id+1, 0.0);
            unfiltered.resize(id+1, 0.0);
        }

        storage[id] = (beta / 1000.0f) * (storage[id] + in - unfiltered[id]);
        unfiltered[id] = in;
        return storage[id];
    }
    static vector<float> unfiltered;
    static vector<float> storage;
};

template<typename FilterPolicy>
class BandFilter : private FilterPolicy {
public:
    /* For each call, increment id of 3 * data.size() to avoid overlapping */ 
    static void process(uint16_t id, vector<Vec3>& data) {
        for(size_t i = 0; i < data.size(); i++)
            process(id + i * 3, data[i]);
    }

    /* For each call, increment id of data.size() to avoid overlapping */
    static void process(uint16_t id, vector<float>& data) {
        for(size_t i = 0; i < data.size(); i++)
            FilterPolicy::process(id + i, data[i]);
    }

    static void printStorage() {
        printf("Lowpass: [");
        for(const float& f : FilterPolicy::storage)
            printf("%5.2f ", f);
        printf("]\n");
    }

    static void process(uint16_t id, Vec3& data) {
        data.setX(FilterPolicy::process(id + 0, data.getX()));
        data.setY(FilterPolicy::process(id + 1, data.getY()));
        data.setZ(FilterPolicy::process(id + 2, data.getZ()));
    }
};

template<uint32_t alpha> vector<float> LowPassPolicy<alpha>::storage;
template<uint32_t beta> vector<float> HighPassPolicy<beta>::storage;
template<uint32_t beta> vector<float> HighPassPolicy<beta>::unfiltered;

#define VECARRAY_FILTER(id, array) do {               \
    BandFilter<LowPassPolicy<lp_alpha> >::process(id, array);    \
    id += array.size() * 3;                           \
} while(0)

#define FLOATARRAY_FILTER(id, array) do {             \
    BandFilter<LowPassPolicy<lp_alpha> >::process(id, array);    \
    id += array.size();                               \
} while(0)

#define COPYVEC(arr,v) do { \
    arr[0] = v.getX();      \
    arr[1] = v.getY();      \
    arr[2] = v.getZ();      \
} while(0)

int main() {
    Thread::sleep(100);
    sDemoBoard->init();
    
    //Ethernet setup
#ifdef ENABLE_ETHERNET
    const uint8_t ipAddr[] = {192,168,1,30}; // Device's IP address
    const uint8_t mask[] = {255,255,255,0};  // Subnet mask
    
    const uint8_t destIp[] = {192,168,1,4};  // Destination IP address
    const uint16_t destPort = 1234;          // Destination port
            
    printf("[Ethernet] Starting...\n");
    W5200& eth = W5200::instance();
    eth.setIpAddress(ipAddr);
    eth.setSubnetMask(mask);
    printf("[Ethernet] Got addr\n");
    
    //This socket listens on UDP port 2020
    //UdpManager::getInstance().setTxPort(2020);    
    printf("[Ethernet] Socket ready\n");
#endif

    Vec3 orientation;
    Vec3 angularvel;
    int ignore_ctr = 100;
    int led_status = 0x200;

    Leds::init();
    while(1) {
        sDemoBoard->update();
        Thread::sleep(20);

        vector<Vec3> accels = sDemoBoard->getAccel();
        vector<Vec3> rots =   sDemoBoard->getRotation();
        vector<Vec3> compsv = sDemoBoard->getCompass();
        vector<float> temps = sDemoBoard->getTemperature();
        vector<float> humid = sDemoBoard->getHumidity();
        vector<float> press = sDemoBoard->getPressure();

        uint16_t id = 0;
        
        VECARRAY_FILTER(id, accels);
        VECARRAY_FILTER(id, rots);
        BandFilter<HighPassPolicy<hp_beta> >::process(0, rots);
        VECARRAY_FILTER(id, compsv);
        FLOATARRAY_FILTER(id, temps);
        FLOATARRAY_FILTER(id, humid);
        FLOATARRAY_FILTER(id, press);

        Vec3 acc = averageVec(accels);
        Vec3 rot = averageVec(rots);
        Vec3 comps = averageVec(compsv);
        float tempm = averageFloat(temps);
        float humim = averageFloat(humid);
        float presm = averageFloat(press);

        pkt_t packet;
        COPYVEC(packet.accel, acc);
        COPYVEC(packet.gyro, rot);
        COPYVEC(packet.compass, comps);
        COPYVEC(packet.orientation, orientation);
        packet.temperature = tempm;
        packet.humidity = humim;
        packet.pressure = presm;

        Leds::set(led_status);
        if(ignore_ctr == 0) {
            led_status <<= 1;
            if(led_status == 0x400)
                led_status = 0x04;
            angularvel  += rot * 0.2f;
            orientation += angularvel * 0.2f;
            BandFilter<HighPassPolicy<999> >::process(4, orientation);
        } else {
            led_status ^= 0x200;
            --ignore_ctr;
            if(ignore_ctr == 0)
                led_status = 0x04;
        }

#ifdef ENABLE_ETHERNET
        sock.sendPacketTo(destIp,destPort,&packet,sizeof(packet));
#endif

        printf( "[%5.2f %5.2f %5.2f] "
                "[%5.2f %5.2f %5.2f] "
                "[%d %5.2f %5.2f %5.2f] "
                "[%5.2f %5.2f %5.2f] " 
                "[%5.2f %5.2f %5.2f] \r",
                acc.getX(), acc.getY(), acc.getZ(),
                rot.getX(), rot.getY(), rot.getZ(), ignore_ctr,
                orientation.getX(), orientation.getY(), orientation.getZ(),
                comps.getX(), comps.getY(), comps.getZ(),
                tempm, humim, presm);

        fflush(stdout);
    }

    // Bye.
    while(1) { Thread::sleep(1000); }
}
