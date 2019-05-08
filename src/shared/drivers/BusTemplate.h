/* Bus base class
 *
 * Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Illya Dudchenko, Matteo Michele Piazzolla, Silvano Seva,
 *          Alain Carlucci
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

#ifndef BUSTEMPLATE_H
#define BUSTEMPLATE_H

#include <Singleton.h>
#include <drivers/spi/SensorSpi.h>
#include <miosix.h>
#include <stdint.h>
#include <stdio.h>
#include <vector>

#include "i2c/stm32f2_f4_i2c.h"

using std::vector;

static const int csDelay = 20;

template <unsigned N, class GpioMosi, class GpioMiso, class GpioSclk>
class BusSPI : public Singleton<BusSPI<N, GpioMosi, GpioMiso, GpioSclk> >
{
    friend class Singleton<BusSPI<N, GpioMosi, GpioMiso, GpioSclk> >;
    typedef Singleton<BusSPI<N, GpioMosi, GpioMiso, GpioSclk> > SingletonType;

public:
    inline static void write(const void* buffer, size_t len)
    {
        SingletonType::getInstance()->_write(buffer, len);
    }
    inline static void write(uint8_t byte)
    {
        SingletonType::getInstance()->_write(byte);
    }

    inline static void init()
    {
        SingletonType::getInstance();
    }

    // inline static void init(uint8_t cpol, uint8_t cpha)
    // {
    //     _cpol = cpol;
    //     _cpha = cpha;
    //     SingletonType::getInstance();
    // }

    inline static int read(void* buffer, size_t max_len)
    {
        return SingletonType::getInstance()->_read(buffer, max_len);
    }

    inline static uint8_t read()
    {
        return SingletonType::getInstance()->_read();
    }

    /**
     * Performs a full duplex transmission.
     * The provided buffer is written on the bus and its content is then
     * replaced by the received data.
     *
     * @param buf buffer containing the data to send & receive
     */
    inline static void transfer(uint8_t* buf, size_t max_len)
    {
        SingletonType::getInstance()->_transfer(buf, max_len);
    }

    /**
     * Performs a full duplex transmission.
     * The provided byte is written on the bus and its content is then
     * replaced by the received data.
     *
     * @param byte Pointer to the byte containing the data to send & receive
     */
    inline static uint8_t transfer(uint8_t* byte)
    {
        return SingletonType::getInstance()->_transfer(byte);
    }

    static inline void setPolarity(uint8_t cpol, uint8_t cpha)
    {
        SingletonType::getInstance()->_setPolarity(cpol,cpha);
    }

private:
    uint8_t _cpol;
    uint8_t _cpha;

    inline void _write(const void* buffer, size_t len) const
    {
        // DMA??
        const uint8_t* buf_ptr = (const uint8_t*)buffer;
        for (unsigned i = 0; i < len; i++)
            _write(*(buf_ptr++));
    }

    inline void _write(uint8_t byte) const
    {
        getSPIAddr(N)->DR = byte;
        while ((getSPIAddr(N)->SR & SPI_SR_RXNE) == 0)
            ;
        volatile uint8_t temp;
        temp = getSPIAddr(N)->DR;
    }

    inline int _read(void* buffer, size_t max_len) const
    {
        // conditional var??
        uint8_t* buf_ptr = (uint8_t*)buffer;
        for (unsigned i = 0; i < max_len; i++)
            *(buf_ptr++) = _read();
        return 0;
    }

    inline uint8_t _read() const
    {
        getSPIAddr(N)->DR = 0;
        while ((getSPIAddr(N)->SR & SPI_SR_RXNE) == 0)
            ;
        return getSPIAddr(N)->DR;
    }

    /**
     * Performs a full duplex transmission.
     * The provided buffer is written on the bus and its content is then
     * replaced by the received data.
     *
     * @param buf buffer containing the data to send & receive
     */
    inline void _transfer(uint8_t* buf, size_t max_len)
    {
        for (size_t i = 0; i < max_len; ++i)
        {
            _transfer(buf + i);
        }
    }

    /**
     * Performs a full duplex transmission.
     * The provided byte is written on the bus and its value is then replaced by
     * the received byte.
     *
     * @param byte pointer to the data to send & receive
     */
    inline uint8_t _transfer(uint8_t* byte)
    {
        getSPIAddr(N)->DR = *byte;
        while ((getSPIAddr(N)->SR & SPI_SR_RXNE) == 0)
            ;
        *byte = getSPIAddr(N)->DR;
        return *byte;
    }

    inline void _setPolarity(uint8_t cpol, uint8_t cpha)
    {
        if(cpol==0)
             getSPIAddr(N)->CR1 &= ~SPI_CR1_CPOL;
        if(cpha==0)
             getSPIAddr(N)->CR1 &= ~SPI_CR1_CPHA;
        if(cpol==1)
             getSPIAddr(N)->CR1 |= SPI_CR1_CPOL;
        if(cpha==1)
             getSPIAddr(N)->CR1 |= SPI_CR1_CPHA;
    }   

    BusSPI()
    {
        // Interrupts are disabled to prevent bugs if more than one threads
        // does a read-modify-write to shared registers at the same time
        {
            miosix::FastInterruptDisableLock dLock;
            IRQenableSPIBus(getSPIAddr(N));
            GpioMosi::mode(miosix::Mode::ALTERNATE);
            GpioMosi::alternateFunction(GetAlternativeFunctionNumber(N));
            GpioMiso::mode(miosix::Mode::ALTERNATE);
            GpioMiso::alternateFunction(GetAlternativeFunctionNumber(N));
            GpioSclk::mode(miosix::Mode::ALTERNATE);
            GpioSclk::alternateFunction(GetAlternativeFunctionNumber(N));
            getSPIAddr(N)->CR1=SPI_CR1_SSM  //No HW cs
                            | SPI_CR1_SSI
                            | SPI_CR1_SPE  //SPI enabled
                           // | SPI_CR1_BR_0
                            | SPI_CR1_BR_1
                            | SPI_CR1_BR_2
                            | SPI_CR1_MSTR; 
            
            if (getSPIAddr(N) == SPI1)
            {
                SPIDriver::instance();
            }
        }
        usleep(csDelay);
    }

    inline static constexpr int GetAlternativeFunctionNumber(int n_spi)
    {
        return n_spi == 1 || n_spi == 2 ? 5 : 6;
    }

    constexpr SPI_TypeDef* getSPIAddr(unsigned n)
    {
        return n == 1 ? SPI1 : n == 2 ? SPI2 : SPI3;
    }

    static inline void IRQenableSPIBus(SPI_TypeDef* spi)
    {
        if (spi == SPI1)
            RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
        else if (spi == SPI2)
            RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
        else if (spi == SPI3)
            RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    }
};

template <class Bus, class GpioCS>
class ProtocolSPI
{
public:
    static inline void init()
    {
        GpioCS::mode(miosix::Mode::OUTPUT);
        GpioCS::high();
        Bus::init();
    }

    /* The standard, single-byte SPI read */
    static uint8_t read(uint8_t reg)
    {
        GpioCS::low();
        reg |= 0x80;
        Bus::write(&reg, sizeof(reg));
        Bus::read(&reg, sizeof(reg));
        GpioCS::high();
        return reg;
    }

    /* The standard, multi-byte SPI read */
    static inline void read(uint8_t reg, uint8_t* buf, int size)
    {
        read_low(reg | 0x80, buf, size);
    }

    /* Read without ask anything */
    static void read(uint8_t* buf, int size)
    {
        GpioCS::low();
        Bus::read(buf, size);
        GpioCS::high();
    }

    static miosix::GpioPin getCSPin() { return GpioCS::getPin(); }

    /* Low-level read: write reg (without | 0x80) and read
     * next N bytes, where N is 'size'
     */
    static void read_low(uint8_t reg, uint8_t* buf, int size)
    {
        GpioCS::low();
        Bus::write(&reg, sizeof(reg));
        Bus::read(buf, size);
        GpioCS::high();
    }

    static void read16(uint16_t reg, uint8_t* buf, int size)
    {   
        uint8_t msb = (uint8_t) (reg >> 8);
        uint8_t lsb = (uint8_t) reg;

        GpioCS::low();
        Bus::write(&msb, sizeof(msb));
        Bus::write(&lsb, sizeof(lsb));        
        Bus::read(buf, size);
        GpioCS::high();
    }

    static void write(uint8_t reg, uint8_t val)
    {
        GpioCS::low();
        Bus::write(&reg, sizeof(reg));
        Bus::write(&val, sizeof(reg));
        GpioCS::high();
    }

    /*
     * Write more than one byte without specifiing reg
     */
    static void write(uint8_t* buf, int size)
    {
        GpioCS::low();
        Bus::write(buf, size);
        GpioCS::high();
    }

    static void write(uint8_t cmd)
    {
        GpioCS::low();
        Bus::write(&cmd, sizeof(cmd));
        GpioCS::high();
    }

    static inline void write(uint8_t reg, uint8_t* buf, int size)
    {
        read_low(reg, buf, size);
    }

    static inline void setPolarity(uint8_t cpol, uint8_t cpha)
    {
        Bus::setPolarity(cpol,cpha);
    }


private:
    ProtocolSPI()                     = delete;
    ~ProtocolSPI()                    = delete;
    ProtocolSPI(const ProtocolSPI& o) = delete;
    ProtocolSPI(ProtocolSPI&& o)      = delete;
    ProtocolSPI& operator=(const ProtocolSPI&) = delete;
    ProtocolSPI& operator=(ProtocolSPI&&) = delete;
};

/*********************************************
 * VERSION OF ProtocolI2C THAT USES HARDWARE *
 * I2C DRIVER                                *
 ********************************************/

template <class Bus>
class ProtocolI2C : public Singleton<ProtocolI2C<Bus> >
{
    friend class Singleton<ProtocolI2C<Bus> >;
    typedef Singleton<ProtocolI2C<Bus> > SingletonType;

public:
    static inline void init() { SingletonType::getInstance(); }

    /**
     * Sends the \param len bytes stored in \param *data buffer
     * to the register specified by \param regAddress
     */
    static inline void write(uint8_t address, uint8_t regAddr, uint8_t* data,
                             uint8_t len)
    {
        SingletonType::getInstance()->writeImpl(address, regAddr, data, len);
    }

    /**
     * Sends the \param len bytes stored in \param *data buffer without
     * specifying a register
     */
    static inline void directWrite(uint8_t address, uint8_t* data, uint8_t len)
    {
        SingletonType::getInstance()->writeImpl(address, data, len);
    }

    /**
     * Reads \param len bytes storing them into \param *data buffer
     * from the register specified by \param regAddress
     */
    static inline void read(uint8_t address, uint8_t regAddr, uint8_t* data,
                            uint8_t len)
    {
        SingletonType::getInstance()->readImpl(address, regAddr, data, len);
    }

    /**
     * Reads \param len bytes storing them into \param *data buffer
     * without specifying the register to read from
     */
    static inline void directRead(uint8_t address, uint8_t* data, uint8_t len)
    {
        SingletonType::getInstance()->directReadImpl(address, data, len);
    }

private:
    Bus& bus = Bus::instance();

    ProtocolI2C() {}

    /* The actual write and read functions implementation.
     * This is a workaround needed to adapt ProtocolI2C class
     * to miosix i2c driver class implementation
     */

    void writeImpl(uint8_t address, uint8_t* data, uint8_t len)
    {
        bus.send(address, reinterpret_cast<void*>(data), len);
    }

    void writeImpl(uint8_t address, uint8_t regAddr, uint8_t* data, uint8_t len)
    {
        vector<uint8_t> buf;
        buf.reserve(len + 1);  // Preallocate to increase performance
        buf.push_back(regAddr);

        memcpy(buf.data() + 1, data, len);

        writeImpl(address, buf.data(), len + 1);
        // bus.send(address, reinterpret_cast<void*>(buf), len + 1);
    }

    void readImpl(uint8_t address, uint8_t regAddr, uint8_t* data, uint8_t len)
    {
        bus.send(address, reinterpret_cast<void*>(&regAddr), 1, false);
        bus.recv(address, reinterpret_cast<void*>(data), len);
    }

    void directReadImpl(uint8_t address, uint8_t* data, uint8_t len)
    {
        bus.recv(address, reinterpret_cast<void*>(data), len);
    }

    ProtocolI2C(const ProtocolI2C& o)  = delete;
    ProtocolI2C(const ProtocolI2C&& o) = delete;
    ProtocolI2C& operator              =(const ProtocolI2C& other);
    ProtocolI2C& operator              =(const ProtocolI2C&& other);
};

#endif  // BUSTEMPLATE_H
