/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Riccardo Musso
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

#define SENSOR_LIS3DSH_STATS_TEST 1

#include <Common.h>
#include <drivers/spi/SPIDriver.h>
#include <math/Stats.h>
#include <miosix.h>

#if SENSOR_LIS3DSH_STATS_TEST
#include "sensors/LIS3DSH/LIS3DSH.h"
#endif

#include "sensors/calibration/Calibration.h"

using namespace Boardcore;
using namespace miosix;

/* using volatile so that the compiler won't optimize out the variable */
volatile AccelerometerData testData;

/*
 * For future readers: new lines are inserted so that each line does no exceed
 * 57 characters (tab counts as 8 + 49 chars).
 */
const char* helpInfo =
    "Here are all available commands:\n\n"
    "  ?\n\tDisplay this information.\n\n"
    "  olist\n\tList which of the orientations are still to sample.\n\n"
    "  oshow\n\tDescribe the current orientation.\n\n"
    "  onext\n\tGo to the next orientation of the list (marking\n\tthe current "
    "one as sampled.\n\n"
    "  oset [idx]\n\tManually sets the current orientation to the "
    "[idx]\n\tone of the list given by 'olist'.\n\n"
    "  take [n]\n\tTake [n] samples from the sensor and use them "
    "to\n\tcalculate the descriptive values such as min, max,"
    "\n\tvariance, etc. Note: old values will be used unless\n\t'discard' "
    "command is called.\n\n"
    "  setsleep [millis]\n\tSet the sleep time between samples to "
    "[millis].\n\tThe default sleep time value is 0.\n\n"
    "  discard\n\tReset all stats about the current orientation.\n\n"
    "  min\n\tShow the minimum value measured on each axis.\n\n"
    "  max\n\tShow the maximum value.\n\n"
    "  mean\n\tShow the mean.\n\n"
    "  stddev\n\tShow the standard deviation.\n\n"
    "  samples\n\tShow how many samples are used for the calculation\n\t"
    "of the descriptive values (for current orientation\n\tonly).\n\n";

int main()
{
    SPIBus bus(SPI1);

    GpioPin spi_sck(GPIOA_BASE, 5);
    GpioPin spi_miso(GPIOA_BASE, 6);
    GpioPin spi_mosi(GPIOA_BASE, 7);
    GpioPin cs(GPIOE_BASE, 3);

    {
        miosix::FastInterruptDisableLock dLock;

        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;  // Enable SPI1 bus

        spi_sck.mode(miosix::Mode::ALTERNATE);
        spi_sck.alternateFunction(5);
        spi_miso.mode(miosix::Mode::ALTERNATE);
        spi_miso.alternateFunction(5);
        spi_mosi.mode(miosix::Mode::ALTERNATE);
        spi_mosi.alternateFunction(5);

        cs.mode(miosix::Mode::OUTPUT);
    }
    cs.high();

    LIS3DSH sensor(bus, cs, sensor.ODR_100_HZ, sensor.UPDATE_AFTER_READ_MODE,
                   sensor.FULL_SCALE_4G);
    sensor.init();

    const AxisOrthoOrientation orientations[] = {
        {Direction::POSITIVE_X, Direction::POSITIVE_Y},
        {Direction::POSITIVE_Y, Direction::POSITIVE_Z},
        {Direction::POSITIVE_Z, Direction::POSITIVE_X},
        {Direction::POSITIVE_Y, Direction::POSITIVE_X},
        {Direction::POSITIVE_Z, Direction::POSITIVE_Y},
        {Direction::POSITIVE_X, Direction::POSITIVE_Z},
    };

    int selected = 0, sleepTime = 0;
    char input[30];

    Stats xAxis[6], yAxis[6], zAxis[6];

    printf(
        "Welcome to the sensor stats tool.\n"
        "Commands are given via serial input. Type ? to see accepted "
        "commands\n");

    while (1)
    {
        printf("> ");
        fgets(input, 30, stdin);

        if (!strncmp("?", input, 1))
        {
            printf("%s", helpInfo);
        }
        else if (!strncmp("olist", input, 5))
        {
            printf("Here are all the recognized orientations:\n");

            for (int i = 0; i < 6; i++)
            {
                const char* xLabel =
                    humanFriendlyDirection[(uint8_t)orientations[i].xAxis];
                const char* yLabel =
                    humanFriendlyDirection[(uint8_t)orientations[i].yAxis];

                printf(" %d. X towards %s, Y towards %s (%d samples)", i,
                       xLabel, yLabel, xAxis[i].getStats().nSamples);

                if (selected == i)
                    printf(" [selected]");
                printf("\n");
            }
        }
        else if (!strncmp("oshow", input, 5))
        {
        iLoveSpaghettiCode:
            const char* xLabel =
                humanFriendlyDirection[(uint8_t)orientations[selected].xAxis];
            const char* yLabel =
                humanFriendlyDirection[(uint8_t)orientations[selected].yAxis];
            printf("Current orientation is: X towards %s, Y towards %s\n",
                   xLabel, yLabel);

            printf("You took %d samples for each axis on this orientation.\n",
                   xAxis[selected].getStats().nSamples);
        }
        else if (!strncmp("onext", input, 5))
        {
            selected++;

            printf("Switching to orientation n. %d\n", selected);
            goto iLoveSpaghettiCode;
        }
        else if (!strncmp("oset", input, 4))
        {
            int idx;
            if (!sscanf(input, "oset %d", &idx) || idx < 0 || idx > 6)
            {
                printf("oset: invalid argument. %d \n", idx);
            }
            else
            {
                selected = idx;

                printf("Switching to orientation n. %d\n", selected);
                goto iLoveSpaghettiCode;
            }
        }
        else if (!strncmp("take", input, 4))
        {
            int n;
            if (!sscanf(input, "take %d", &n) || n <= 0)
            {
                printf("take: invalid argument.\n");
            }
            else
            {
                for (int i = 0; i < n; ++i)
                {
                    sensor.sample();
                    auto data = sensor.getLastSample();

                    xAxis[selected].add(data.accel_x);
                    yAxis[selected].add(data.accel_y);
                    zAxis[selected].add(data.accel_z);

                    printf("Added sample: %f, %f, %f\n", data.accel_x,
                           data.accel_y, data.accel_z);

                    if (sleepTime > 0)
                    {
                        Thread::sleep(sleepTime);
                    }
                }
            }
        }
        else if (!strncmp("setsleep", input, 8))
        {
            int newSleep;
            if (!sscanf(input, "setsleep %d", &newSleep) || newSleep < 0)
            {
                printf("setsleep: invalid argument.\n");
            }
            else
            {
                sleepTime = newSleep;
                printf("Set new sleep time to %d milliseconds.\n", newSleep);
            }
        }
        else if (!strncmp("discard", input, 7))
        {
            xAxis[selected].reset();
            yAxis[selected].reset();
            zAxis[selected].reset();

            printf("Data discarded.\n");
        }
        else if (!strncmp("min", input, 3))
        {
            StatsResult xRes = xAxis[selected].getStats(),
                        yRes = yAxis[selected].getStats(),
                        zRes = zAxis[selected].getStats();

            if (xRes.nSamples == 0)
            {
                printf(
                    "Error: not enough samples. Please invoke command "
                    "'take'.\n");
            }
            else
            {
                printf("\t%f, %f, %f\n", xRes.minValue, yRes.minValue,
                       zRes.minValue);
            }
        }
        else if (!strncmp("max", input, 3))
        {
            StatsResult xRes = xAxis[selected].getStats(),
                        yRes = yAxis[selected].getStats(),
                        zRes = zAxis[selected].getStats();

            if (xRes.nSamples == 0)
            {
                printf(
                    "Error: not enough samples. Please invoke command "
                    "'take'.\n");
            }
            else
            {
                printf("\t%f, %f, %f\n",
                       xRes.maxValue, yRes.maxValue, zRes.maxValue);
            }
        }
        else if (!strncmp("mean", input, 3))
        {
            StatsResult xRes = xAxis[selected].getStats(),
                        yRes = yAxis[selected].getStats(),
                        zRes = zAxis[selected].getStats();

            if (xRes.nSamples == 0)
            {
                printf(
                    "Error: not enough samples. Please invoke command "
                    "'take'.\n");
            }
            else
            {
                printf("\t%f, %f, %f\n",
                       xRes.mean, yRes.mean, zRes.mean);
            }
        }
        else if (!strncmp("stddev", input, 3))
        {
            StatsResult xRes = xAxis[selected].getStats(),
                        yRes = yAxis[selected].getStats(),
                        zRes = zAxis[selected].getStats();

            if (xRes.nSamples == 0)
            {
                printf(
                    "Error: not enough samples. Please invoke command "
                    "'take'.\n");
            }
            else
            {
                printf(
                    "\t%f, %f, %f\n",
                    xRes.stdev, yRes.stdev, zRes.stdev);
            }
        }
        else if (!strncmp("samples", input, 7))
        {
            printf("Taken %d samples on the current orientation.\n",
                   xAxis[selected].getStats().nSamples);
        }
        else
        {
            printf("Unrecognized command. Type ? for help.\n");
        }
    }
}
