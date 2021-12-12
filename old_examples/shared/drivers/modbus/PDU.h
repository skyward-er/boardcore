/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Author: Silvano Seva
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

#include <stddef.h>

#include "Common.h"

class PDU
{
public:
    /**
     * Create a new PDU structure
     * @param fCode: PDU's function code
     * @param data: pointer to data to be put into PDU's data field
     * @param dataSize: size of PDU's data field
     */
    PDU(uint8_t fCode, const uint8_t const* data, uint8_t dataSize);

    /**
     * Default constructor does nothing
     */
    PDU();
    ~PDU();

    /**
     * @return PDU's function code
     */
    uint8_t funcCode() const;

    /**
     * @return std::pair: first field is data size, the second one is
     * a uint8_t const* to internal data.
     */
    std::pair<uint8_t, uint8_t const*> data() const;

private:
    uint8_t fuCode;
    uint8_t pSize;
    uint8_t* pData;

    PDU(const PDU& other);
    PDU& operator=(const PDU& other);
    bool operator==(const PDU& other);
};
