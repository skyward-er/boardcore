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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef SENSOR_SPI_H
#define SENSOR_SPI_H

#include <Common.h>

class SPIRequest;

enum DMAFIFOStatus
{
    DFS_UNK = -1,  // Unk
    DFS_EE  = 0,   // x == 0
    DFS_10  = 1,   // 0  <  x < 25
    DFS_25  = 2,   // 25 <= x < 50
    DFS_50  = 3,   // 50 <= x < 75
    DFS_75  = 4,   // 75 <= x < 100
    DFS_100 = 5,   // x == 100
};

class SPIDriver
{
public:
    static SPIDriver& instance();

    bool transaction(std::vector<SPIRequest>& requests);

    DMAFIFOStatus getTxFIFOStatus() const
    {
        return intToFIFOStatus((DMA2_Stream5->FCR & DMA_SxFCR_FS) >> 3);
    }

    DMAFIFOStatus getRxFIFOStatus() const
    {
        return intToFIFOStatus((DMA2_Stream0->FCR & DMA_SxFCR_FS) >> 3);
    }
    uint32_t getFIFOFaultCtr() const;

private:
    SPIDriver();

    SPIDriver(const SPIDriver&);
    SPIDriver& operator=(const SPIDriver&);

    inline void enableDMA() { SPI1->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN; }

    inline void disableDMA() { SPI1->CR2 = 0; }

    inline DMAFIFOStatus intToFIFOStatus(uint8_t s) const
    {
        switch (s)
        {
            case 0:
                return DFS_10;
            case 1:
                return DFS_25;
            case 2:
                return DFS_50;
            case 3:
                return DFS_75;
            case 4:
                return DFS_EE;
            case 5:
                return DFS_100;
        }
        return DFS_UNK;
    }

    pthread_mutex_t mutex;
};

class SPIRequest
{
public:
    SPIRequest(int id, miosix::GpioPin pin) : mId(id), chipSelect(pin) {}

    SPIRequest(int id, miosix::GpioPin pin, const std::vector<uint8_t>& data)
        : mId(id), chipSelect(pin)
    {
        setDataToPeripheral(data);
    }

    void setDataToPeripheral(const uint8_t* data, size_t size)
    {
        toPeripheral.resize(size);
        memcpy(toPeripheral.data(), data, size);
        fromPeripheral.resize(size);
    }

    void setDataToPeripheral(const std::vector<uint8_t>& data)
    {
        toPeripheral = data;
        fromPeripheral.resize(data.size());
    }

    uint8_t readResponseByteFromPeripheral(int index) const
    {
        return fromPeripheral.at(index);
    }

    const std::vector<uint8_t>& readResponseFromPeripheral() const
    {
        return fromPeripheral;
    }

    bool empty() const { return toPeripheral.empty(); }

    int id() const { return mId; }

    void IRQbeginTransaction();

    void IRQendTransaction();

private:
    const int mId;
    miosix::GpioPin chipSelect;
    std::vector<uint8_t> toPeripheral;
    std::vector<uint8_t> fromPeripheral;
};

#endif  // SENSOR_SPI_H
