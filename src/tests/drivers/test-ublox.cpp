#include <Common.h>
#include <drivers/gps/ublox/Gps.h>
#include <drivers/serial.h>
#include <fcntl.h>
#include <filesystem/file_access.h>

using namespace miosix;

int main()
{
    // Open GPS serial on USART2 (PA2, PA3)
    intrusive_ref_ptr<DevFs> devFs = FilesystemManager::instance().getDevFs();
    devFs->addDevice("gps",
                     intrusive_ref_ptr<Device>(new STM32Serial(2, 38400)));

    TimestampTimer::enableTimestampTimer();

    // Keep GPS baud rate at default for easier testing
    Gps gps(38400);
    struct UbloxGPSData dataGPS;

    printf("init gps: %d\n", gps.init());
    Thread::sleep(200);
    gps.start();
    printf("selftest gps: %d\n", gps.selfTest());

    gps.sendSBASMessage();

    while (1)
    {
        Thread::sleep(2000);
        gps.sample();

        dataGPS = gps.getLastSample();
        printf(
            "timestamp: %.3f, fix: %d t: %lld lat: %f lon: %f height: %f nsat: "
            "%d speed: %f "
            "velN: %f velE: "
            "%f track %f\n",
            (float)dataGPS.gps_timestamp / 1000000, dataGPS.fix,
            dataGPS.gps_timestamp, dataGPS.latitude, dataGPS.longitude,
            dataGPS.height, dataGPS.num_satellites, dataGPS.speed,
            dataGPS.velocity_north, dataGPS.velocity_east, dataGPS.track);
    }
}
