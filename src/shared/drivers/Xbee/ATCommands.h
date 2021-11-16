/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#pragma once

#include "APIFrames.h"
#include "Xbee.h"

namespace Boardcore
{

namespace Xbee
{

/**
 * @brief Enables or disable the specified channels. Channel 9 and 24 are always
 * disabled due to regulatory limitations.
 * See datasheet for the list of channel frequencies.
 *
 * @param xbee Reference to an xbee object
 * @param channels channel mask
 * @return True if the command was executed successfully
 */
bool setChannelMask(Xbee& xbee, uint32_t mask, unsigned int timeout = 1000)
{
    ATCommandResponseFrame response;

    mask = swapBytes32(mask);

    return xbee.sendATCommand("CM", &response,
                              reinterpret_cast<uint8_t*>(&mask), 4, timeout);
}

/**
 * @brief Enables or disable the specified channels. Channel 9 and 24 are always
 * disabled due to regulatory limitations.
 * See datasheet for the list of channel frequencies.
 *
 * @param xbee Reference to an xbee object
 * @param channels channel mask array
 *
 * @return True if the command was executed successfully
 */
bool setChannelMask(Xbee& xbee, bool channels[30], unsigned int timeout = 1000)
{
    uint32_t val = 0;

    for (int i = 0; i < 30; i++)
    {
        val &= ((uint32_t)channels[i]) << i;
    }

    return setChannelMask(xbee, val, timeout);
}

/**
 * @brief Enable communication on all available channels. Equivalent to calling
 * setChannelMask(xbee, 0x3EFFFDFF).
 *
 * @param xbee Reference to an xbee object
 * @return True if the command was executed successfully
 */
bool enableAllChannels(Xbee& xbee, unsigned int timeout = 1000)
{
    return setChannelMask(xbee, 0x3EFFFDFF, timeout);
}

/**
 * @brief Disables frequency hopping by allowing communication only on channel
 * #29. Equivalent to calling setChannelMask(xbee, 0x20000000).
 *
 * @param xbee Reference to an xbee object
 */
bool disableFrequencyHopping(Xbee& xbee, unsigned int timeout = 1000)
{
    return setChannelMask(xbee, 0x20000000, timeout);
}

/**
 * @brief Configures the desired xbee data rate.
 *
 * @param xbee Reference to an xbee object
 * @param data_rate_80kbps true for 80kbps, false for 10kbps
 * @return True if the command was executed successfully
 */
bool setDataRate(Xbee& xbee, bool data_rate_80kbps, unsigned int timeout = 1000)
{
    uint8_t param = (uint8_t)data_rate_80kbps;
    ATCommandResponseFrame response;

    return xbee.sendATCommand("BR", &response, &param, 1, timeout);
}

/**
 * @brief Writes parameter values to non-volatile memory so that parameter
 * modifications persist through subsequent resets.
 *
 * @param xbee Reference to an xbee object
 * @return True if the command was executed successfully
 */
bool writeToMemory(Xbee& xbee, unsigned int timeout = 1000)
{
    ATCommandResponseFrame response;
    return xbee.sendATCommand("WR", &response, nullptr, 0, timeout);
}

/**
 * @brief Performs an energy detect scan on all channels
 *
 * @param xbee Reference to an xbee object
 * @param energy_detect_data Pointer to a 30-bytes buffer where energy levels
 * will be stored. Energy levels are in -dBm units.
 * @param duration Scan duration in ms
 * @return True if the command was executed successfully and data stored in
 * energy_detect_data is valid
 */
bool energyDetect(Xbee& xbee, int* energy_detect_data, uint8_t duration,
                  unsigned int timeout = 1000)
{
    ATCommandResponseFrame response;
    if (xbee.sendATCommand("ED", &response, &duration, 1, timeout))
    {
        uint16_t resp_len = response.getCommandDataLength();
        if (resp_len == 30)
        {
            for (int i = 0; i < 30; i++)
            {
                energy_detect_data[i] =
                    (int)(*(response.getCommandDataPointer() + i));
            }

            return true;
        }
    }
    return false;
}

}  // namespace Xbee

}  // namespace Boardcore
