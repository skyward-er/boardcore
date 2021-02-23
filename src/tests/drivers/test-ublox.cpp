#include <Common.h>
#include <drivers/gps/Gps.h>
#include <drivers/serial.h>
#include <filesystem/file_access.h>
#include <drivers/sd_stm32f2_f4.h>
#include <fcntl.h>

using namespace miosix;

int main()
{
    // Open GPS serial on USART2 (PA2, PA3)
    intrusive_ref_ptr<DevFs> devFs = FilesystemManager::instance().getDevFs();
    devFs->addDevice("gps", intrusive_ref_ptr<Device>(new STM32Serial(2, 38400)));

    // Keep GPS baud rate at default for easier testing
    Gps gps(38400);
    struct GPSData dataGPS;

    printf("init gps: %d\n", gps.init());
    Thread::sleep(200);
    gps.start();
    printf("selftest gps: %d\n", gps.selfTest());

    gps.sendSBASMessage();

    while (1)
    {
        Thread::sleep(1000);
        dataGPS = gps.getGpsData();
        printf(
            "fix: %d t: %lld lat: %f lon: %f alt: %f nsat: %d speed: %f velN: %f velE: "
            "%f track: %f\n", dataGPS.fix,
            dataGPS.timestamp, dataGPS.latitude, dataGPS.longitude,
            dataGPS.altitude, dataGPS.numSatellites, dataGPS.speed,
            dataGPS.velocityNorth, dataGPS.velocityEast, dataGPS.track);
    }
}
