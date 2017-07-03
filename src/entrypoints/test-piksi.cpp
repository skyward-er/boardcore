#include <drivers/piksi/piksi.h>
#include <time.h>
#include <iostream>

using namespace std;

#ifdef _MIOSIX

#include <miosix.h>

using namespace miosix;

#endif //_MIOSIX

int main(int argc, char *argv[])
{
    Piksi piksi("/dev/gps");
    for(;;)
    {
        auto gps=piksi.waitForGpsData();
        #ifdef _MIOSIX
        long long now=getTick();
        #else //_MIOSIX
        long long now=clock()/(CLOCKS_PER_SEC/1000);
        #endif //_MIOSIX
        cout<<" t: "<<now-gps.timestamp
            <<" lat: "<<gps.latitude
            <<" lon: "<<gps.longitude
            <<" h: "<<gps.height
            <<" vn: "<<gps.velocityNorth
            <<" ve: "<<gps.velocityEast
            <<" vd: "<<gps.velocityDown
            <<" ns: "<<gps.numSatellites<<endl;
    }
}