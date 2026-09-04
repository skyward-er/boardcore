/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Leonardo Montecchi
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <miosix.h>
#include <units/Length.h>
#include <utils/AltitudeMap/AltitudeQuadMap.h>

#include <iostream>

using namespace miosix;
using namespace Boardcore;
using namespace Units::Length;

int main()
{
    AltitudeQuadMap map("maptree_data.bin");
    if (!map.init())
    {
        std::cerr << "Failed to initialize the altitude map." << std::endl;
        return -1;
    }

    while (true)
    {
        float n_f, e_f;
        std::cout << "Enter coordinates (n, e): ";
        std::cin >> n_f >> e_f;

        Meter n{n_f}, e{e_f};

        auto altitude = map.getClosestGroundAltitude(n, e);

        if (map.isInsideMap(n, e))
        {
            std::cout << "Closest ground altitude: " << altitude << " meters"
                      << std::endl;
        }
        else
        {
            std::cout << "Coordinates are outside the map boundaries: "
                      << altitude << " meters" << std::endl;
        }
    }

    return 0;
}
