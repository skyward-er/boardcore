/***************************************************************************
 *   Copyright (C) 2016 by Terraneo Federico                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/

#include <vector>
#include <miosix.h>
#include <pthread.h>
#include <cstdint>
#include <cstdio>

#ifndef SPI_DRIVER_H
#define	SPI_DRIVER_H

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
    
    void setDataToPeripheral(uint8_t *data, size_t size)
    {
        toPeripheral.resize(size);
        memcpy(toPeripheral.data(),data,size);
        fromPeripheral.resize(size);
    }
    
    void setDataToPeripheral(const std::vector<uint8_t> data)
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
