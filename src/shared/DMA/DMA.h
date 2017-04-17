/* Copyright (c) 2015-2017 Skyward Experimental Rocketry
 * Authors: Federico Terraneo, Alain Carlucci
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

#ifndef SPI_DRIVER_H
#define	SPI_DRIVER_H

#include <Common.h>

class SPIRequest;

class SPIDriver
{
public:
    static SPIDriver& instance();
    
    bool transaction(std::vector<SPIRequest>& requests);
    
private:
    SPIDriver();
    
    SPIDriver(const SPIDriver&);
    SPIDriver& operator= (const SPIDriver&);
    
    pthread_mutex_t mutex;
};

class SPIRequest
{
public:
    SPIRequest(miosix::GpioPin pin) : chipSelect(pin) {}
    
    SPIRequest(miosix::GpioPin pin, const std::vector<uint8_t>& data)
        : chipSelect(pin)
    {
        setDataToPeripheral(data);
    }
    
    void setDataToPeripheral(uint8_t *data, size_t size)
    {
        toPeripheral.resize(size);
        memcpy(toPeripheral.data(),data,size);
        fromPeripheral.resize(size);
    }
    
    void setDataToPeripheral(const std::vector<uint8_t>& data)
    {
        toPeripheral=data;
        fromPeripheral.resize(data.size());
    }
    
    uint8_t readResponseByteFromPeripheral(int index)
    {
        return fromPeripheral.at(index);
    }
    
    std::vector<uint8_t> readResponseFromPeripheral()
    {
        return fromPeripheral;
    }
    
    bool empty()
    {
        return toPeripheral.empty();
    }
    
    void IRQbeginTransaction();
    
    void IRQendTransaction();
    
private:
    miosix::GpioPin chipSelect;
    std::vector<uint8_t> toPeripheral;
    std::vector<uint8_t> fromPeripheral;
};

#endif //SPI_DRIVER_H
