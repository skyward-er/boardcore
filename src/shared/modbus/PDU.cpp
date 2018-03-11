/* Modbus protocol PDU data structure
 *
 * Copyright (c) 2017 Skyward Experimental Rocketry
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#include "PDU.h"

PDU::PDU() : fuCode(0), pSize(0), pData(nullptr) {

}

PDU::PDU(uint8_t fCode, const uint8_t* data, uint8_t dataSize) : fuCode(fCode),
                                                             pSize(dataSize) {
    
    #ifndef __NO_EXCEPTIONS
    try {
        pData = new uint8_t[pSize];
    }catch(std::bad_alloc& exc)
    {
        std::cout << "bad alloc! " << exc.what() << std::endl;
        pData = nullptr;
    }
    #else
    pData = new (std::nothrow) uint8_t[pSize];
    #endif
    
    std::memcpy(pData,data,pSize);
}

PDU::~PDU() {
    
    if(pData != nullptr) {
        delete[] pData;
    }
}

uint8_t PDU::funcCode() const {
    
    return fuCode;
}

std::pair< uint8_t, uint8_t const* > PDU::data() const {    
    
    return std::pair< uint8_t, const uint8_t* >(pSize, pData);    
}

