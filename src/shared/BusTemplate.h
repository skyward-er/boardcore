/* Bus base class
 *
 * Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Illya Dudchenko, Matteo Michele Piazzolla, Silvano Seva, Alain Carlucci
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

#include <stdint.h>
#include <stdio.h>
#include <miosix.h>
#include <Singleton.h>

using namespace std;
using namespace miosix;

#define CS_DELAY 20

template<unsigned N, class GpioMosi, class GpioMiso, class GpioSclk>
class BusSPI : public Singleton< BusSPI<N, GpioMosi, GpioMiso, GpioSclk> > {
    friend class Singleton<BusSPI<N, GpioMosi, GpioMiso, GpioSclk> >;
    typedef Singleton<BusSPI<N, GpioMosi, GpioMiso, GpioSclk> > SingletonType;
public:

    inline static int write(const void* buffer, size_t len) {
        return SingletonType::getInstance()->_write(buffer, len);
    }
    inline static int write(uint8_t byte) {
        return SingletonType::getInstance()->_write(byte);
    }

    inline static void init() { 
        SingletonType::getInstance(); 
    }
    inline static int read(void* buffer, size_t max_len) {
        return SingletonType::getInstance()->_read(buffer, max_len);
    }

    inline static uint8_t read() {
        return SingletonType::getInstance()->_read();
    }
private:
    inline int _write(const void* buffer, size_t len) const {
        // DMA??
        const uint8_t* buf_ptr = (const uint8_t*)buffer;
        for(unsigned i=0;i<len;i++)
            _write(*(buf_ptr++));
        return 0;
    }

    inline void _write(uint8_t byte) const {
        getSPIAddr(N)->DR=byte;
        while((getSPIAddr(N)->SR & SPI_SR_RXNE)==0);
        byte=getSPIAddr(N)->DR;
    }

    inline int _read(void* buffer, size_t max_len) const {
        // conditional var??
        uint8_t* buf_ptr = (uint8_t*)buffer;
        for(unsigned i=0;i<max_len;i++)
            *(buf_ptr++) = _read();
        return 0;
    }

    inline uint8_t _read() const {
        getSPIAddr(N)->DR=0;
        while((getSPIAddr(N)->SR & SPI_SR_RXNE)==0);
        return getSPIAddr(N)->DR;
    }

    BusSPI() {
        GpioMosi::mode(Mode::ALTERNATE);
        GpioMosi::alternateFunction(GetAlternativeFunctionNumber(N));
        GpioMiso::mode(Mode::ALTERNATE);
        GpioMiso::alternateFunction(GetAlternativeFunctionNumber(N));
        GpioSclk::mode(Mode::ALTERNATE);
        GpioSclk::alternateFunction(GetAlternativeFunctionNumber(N));
        usleep(CS_DELAY);
        enableSPIBus(getSPIAddr(N));
        getSPIAddr(N)->CR1 = SPI_CR1_SSM
                           | SPI_CR1_SSI
                           | SPI_CR1_MSTR
        //                   | SPI_CR1_BR_0 
        //                   | SPI_CR1_BR_1
                           | SPI_CR1_BR_2
                           | SPI_CR1_SPE;
    }

    inline static constexpr int GetAlternativeFunctionNumber(int n_spi) {
        return n_spi == 1 || n_spi == 2 ? 5 : 6;
    }

    constexpr SPI_TypeDef* getSPIAddr(unsigned n) {
        return  n==1 ? SPI1 :
                n==2 ? SPI2 : SPI3;
    }

    static inline void enableSPIBus(SPI_TypeDef* spi) {
        if(spi == SPI1)
            RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
        else if(spi == SPI2)
            RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
        else if(spi == SPI3)
            RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    }
};

template<class Bus, class GpioCS>
class ProtocolSPI {
public:
    static inline void init() {
        GpioCS::mode(Mode::OUTPUT);
        GpioCS::high();
        Bus::init();
    }

	/* The standard, single-byte SPI read */
    static uint8_t read(uint8_t reg) {
        GpioCS::low();
        reg |= 0x80;
        Bus::write(&reg, sizeof(reg));
        Bus::read(&reg, sizeof(reg));
        GpioCS::high();
        return reg;
    }
    
	/* The standard, multi-byte SPI read */
    static inline void read(uint8_t reg, uint8_t *buf, int size) {
		read_low(reg | 0x80, buf, size);
    }

	/* Read without ask anything */
	static void read(uint8_t *buf, int size) {
		GpioCS::low();
		Bus::read(buf, size);
		GpioCS::high();
	}

	/* Low-level read: write reg (without | 0x80) and read 
	 * next N bytes, where N is 'size'
     */
	static void read_low(uint8_t reg, uint8_t *buf, int size) {
		GpioCS::low();
		Bus::write(&reg, sizeof(reg));
		Bus::read(buf, size);
		GpioCS::high();
	}
 
    static void write(uint8_t reg, uint8_t val) {
        GpioCS::low();
        Bus::write(&reg, sizeof(reg));
        Bus::write(&val, sizeof(reg));
        GpioCS::high();
    }

    static void write(uint8_t cmd) {
        GpioCS::low();
        Bus::write(&cmd, sizeof(cmd));
        GpioCS::high();
    }

	static inline void write(uint8_t reg, uint8_t *buf, int size) {
	    read_low(reg, buf, size);
	}
private:
    ProtocolSPI() = delete;
    ~ProtocolSPI() = delete;
    ProtocolSPI(const ProtocolSPI& o) = delete;
    ProtocolSPI(const ProtocolSPI&& o) = delete;
    ProtocolSPI& operator=(const ProtocolSPI& other);
    ProtocolSPI& operator=(const ProtocolSPI&& other);
};

/*********************************************
 * MODIFIED VERSION OF ProtocolI2C THAT USES *
 * SOFTWARE I2C DRIVER, TO BE USED UNTIL THE *
 * HARDWARE ONE DOESN'T WORK                 *
 ********************************************/

class ProtocolI2C : public Singleton<ProtocolI2C>{
    friend class Singleton< ProtocolI2C >;
    typedef Singleton< ProtocolI2C > SingletonType;
public:
    
    static inline void init() {
        SingletonType::getInstance();
    }
    
    /**
     * Sends the \param len bytes stored in \param *data buffer 
     * to the register specified by \param regAddress        
     */
    static inline void write(uint8_t address, uint8_t regAddr, uint8_t *data, uint8_t len) {                
        SingletonType::getInstance() -> writeImpl(address,regAddr,data,len);
    }
    
    /** 
     * Reads \param len bytes storing them into \param *data buffer 
     * from the register specified by \param regAddress        
     * The \param sendRegAddr is used in Si7021 driver to disable the transmission of the
     * register address. If set to false the driver will send the slave address in read mode
     * and then it will immediately enter in listening mode
     */
    static inline void read(uint8_t address, uint8_t regAddr, uint8_t *data, uint8_t len, bool sendRegAddr = true) {                
        SingletonType::getInstance() -> readImpl(address,regAddr,data,len,sendRegAddr);
    }

private:

    typedef Gpio<GPIOB_BASE,7> sda;
    typedef Gpio<GPIOB_BASE,8> scl;
    
    typedef SoftwareI2C<sda, scl> i2c;

    ProtocolI2C() { i2c::init(); }
    
    /* The actual write and read functions implementation.
     * This is a workaround needed to adapt ProtocolI2C class
     * to miosix i2c driver class implementation 
     */
    
    void writeImpl(uint8_t address, uint8_t regAddr, uint8_t *data, uint8_t len) {
        
        i2c::sendStart();
        i2c::send(address & 0xfe);  //set LSB to zero
        i2c::send(regAddr);
        
        for(int i=0; i<len; i++)
            i2c::send(data[i]);
        
        i2c::sendStop();
    }
    
    void readImpl(uint8_t address, uint8_t regAddr, uint8_t *data, uint8_t len, bool sendRegAddr) {
                        
        if(sendRegAddr){
            i2c::sendStart();
            i2c::send(address & 0xfe);
            i2c::send(regAddr);        
        }
        
        i2c::init();
        i2c::sendStart();
        i2c::send(address | 0x01);
        
        for(int i = 0; i<len-1; i++)
            data[i] = i2c::recvWithAck();
        
        data[len-1] = i2c::recvWithNack();
        i2c::sendStop();
    }

    ProtocolI2C(const ProtocolI2C& o) = delete;
    ProtocolI2C(const ProtocolI2C&& o) = delete;
    ProtocolI2C& operator=(const ProtocolI2C& other);
    ProtocolI2C& operator=(const ProtocolI2C&& other);
};



// template<class Bus>
// class ProtocolI2C : public Singleton<ProtocolI2C<Bus> >{
//     friend class Singleton< ProtocolI2C<Bus> >;
//     typedef Singleton< ProtocolI2C<Bus> > SingletonType;
// public:
//     
//     static inline void init() {
//         SingletonType::getInstance();
//     }
//     
//     /**
//      * Sends the \param len bytes stored in \param *data buffer 
//      * to the register specified by \param regAddress        
//      */
//     static inline void write(uint8_t address, uint8_t regAddr, uint8_t *data, uint8_t len) {                
//         SingletonType::getInstance() -> writeImpl(address,regAddr,data,len);
//     }
//     
//     /** 
//      * Reads \param len bytes storing them into \param *data buffer 
//      * from the register specified by \param regAddress        
//      */
//     static inline void read(uint8_t address, uint8_t regAddr, uint8_t *data, uint8_t len) {                
//         SingletonType::getInstance() -> readImpl(address,regAddr,data,len);
//     }
// 
// private:
//     Bus& bus = Bus::instance();
// 
//     ProtocolI2C() { }
//     
//     /* The actual write and read functions implementation.
//      * This is a workaround needed to adapt ProtocolI2C class
//      * to miosix i2c driver class implementation 
//      */
//     
//     void writeImpl(uint8_t address, uint8_t regAddr, uint8_t *data, uint8_t len) {
//         uint8_t buf[len+1];     //pack register address and payload
//         buf[0] = regAddr;
//         
//         memcpy(buf+1, data, len);
//         
//         for(int i=0; i<len+1; i++)
//             printf("%x at %d\n",buf[i],i);
//         
//         bus.send(address, reinterpret_cast<void*>(buf), len+1);
//     }
//     
//     void readImpl(uint8_t address, uint8_t regAddr, uint8_t *data, uint8_t len) {
//         bus.send(address,reinterpret_cast<void*>(&regAddr),1);
// //         usleep(50);
//         bus.recv(address,reinterpret_cast<void*>(data),len);
//     }
// 
//     ProtocolI2C(const ProtocolI2C& o) = delete;
//     ProtocolI2C(const ProtocolI2C&& o) = delete;
//     ProtocolI2C& operator=(const ProtocolI2C& other);
//     ProtocolI2C& operator=(const ProtocolI2C&& other);
// };

#endif // BUSTEMPLATE_H
