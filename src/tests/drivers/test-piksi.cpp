/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Authors: Federico Terraneo
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

#include <drivers/old_examples/piksi/piksi.h>
#include <time.h>
#include <iostream>
using namespace std;

#ifdef _MIOSIX

#include <miosix.h>
using namespace miosix;

#endif  //_MIOSIX

// Polling test, usng getGpsData()

int main()
{
#ifdef _MIOSIX

    Piksi piksi("/dev/gps");
#else   //_MIOSIX
    Piksi piksi("/dev/ttyUSB0");
#endif  //_MIOSIX
    for (;;)
    {
        Thread::sleep(200);
        try
        {
            auto gps = piksi.getGpsData();
#ifdef _MIOSIX
            long long now = getTick();
#else   //_MIOSIX
            long long now = clock() / (CLOCKS_PER_SEC / 1000);
#endif  //_MIOSIX
            cout << " t: " << gps.timestamp << " lat: " << gps.latitude
                 << " lon: " << gps.longitude << " h: " << gps.height
                 << " vn: " << gps.velocityNorth << " ve: " << gps.velocityEast
                 << " vd: " << gps.velocityDown << " ns: " << gps.numSatellites
                 << " now: " << now;
        }
        catch (...)
        {
            cout << "---" << endl;
        }
    }
}

// Blocking wait test, using waitForGpsData()

// int main()
// {
//     #ifdef _MIOSIX
//     Piksi piksi("/dev/gps");
//     #else //_MIOSIX
//     Piksi piksi("/dev/ttyUSB0");
//     #endif //_MIOSIX
//
//     for(;;)
//     {
//         auto gps=piksi.waitForGpsData();
//         #ifdef _MIOSIX
//         long long now=getTick();
//         #else //_MIOSIX
//         long long now=clock()/(CLOCKS_PER_SEC/1000);
//         #endif //_MIOSIX
//         cout<<" t: "<<now-gps.timestamp
//             <<" lat: "<<gps.latitude
//             <<" lon: "<<gps.longitude
//             <<" h: "<<gps.height
//             <<" vn: "<<gps.velocityNorth
//             <<" ve: "<<gps.velocityEast
//             <<" vd: "<<gps.velocityDown
//             <<" ns: "<<gps.numSatellites<<endl;
//     }
// }
