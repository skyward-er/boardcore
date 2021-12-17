/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <stddef.h>
#include <stdint.h>

namespace Boardcore
{

namespace CRCUtils
{

/**
 * @brief Generic 16bit wide CRC calculation.
 *
 * @tparam poly CRC polynomial
 * @tparam init CRC initial value
 * @param buffer Data buffer
 * @param count Buffer length in byte
 * @return Computed CRC value
 */
template <uint16_t poly, uint16_t init>
uint16_t generiCrc16(const uint8_t *buffer, size_t count)
{
    uint16_t crc = init;

    for (size_t i = 0; i < count; i++)
    {
        crc ^= static_cast<uint16_t>(buffer[i]) << 8;

        for (uint8_t ii = 0; ii < 8; ii++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
    }

    return crc;
}

/**
 * @brief CRC-16/CCITT-FALSE.
 *
 * @param buffer Data buffer
 * @param count Buffer length in byte
 * @return unsigned short Computed CRC value
 */
const auto crc16 = generiCrc16<0x1021, 0xffff>;

}  // namespace CRCUtils

}  // namespace Boardcore
