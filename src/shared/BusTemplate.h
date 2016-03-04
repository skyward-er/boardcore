#ifndef BUSTEMPLATE_H
#define BUSTEMPLATE_H

#include <stdint.h>
#include "miosix.h"
#include "singleton.h"

using namespace std;
using namespace miosix;

#define CS_DELAY 20

template<unsigned N, class GpioMosi, class GpioMiso, class GpioSclk>
class BusSPI : public Singleton< BusSPI<N, GpioMosi, GpioMiso, GpioSclk> > {

    friend class Singleton<BusSPI<N, GpioMosi, GpioMiso, GpioSclk> >;
    typedef Singleton<BusSPI<N, GpioMosi, GpioMiso, GpioSclk> > SingletonType;
public:
    inline static int Write(const void* buffer, size_t len) {
        return SingletonType::GetInstance()->_write(buffer, len);
    }
    inline static void Init() { 
        SingletonType::GetInstance(); 
    }
    inline static int Read(void* buffer, size_t max_len) {
        return SingletonType::GetInstance()->_read(buffer, max_len);
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
        GetSpiAddr(N)->DR=byte;
        while((GetSpiAddr(N)->SR & SPI_SR_RXNE)==0);
        byte=GetSpiAddr(N)->DR;
    }

    inline int _read(void* buffer, size_t max_len) const {
        // conditional var??
        uint8_t* buf_ptr = (uint8_t*)buffer;
        for(unsigned i=0;i<max_len;i++)
            *(buf_ptr++) = _read();
        return 0;
    }

    inline uint8_t _read() const {
        GetSpiAddr(N)->DR=0;
        while((GetSpiAddr(N)->SR & SPI_SR_RXNE)==0);
        return GetSpiAddr(N)->DR;
    }

    BusSPI() {
        GpioMosi::mode(Mode::ALTERNATE);
        GpioMosi::alternateFunction(GetAlternativeFunctionNumber(N));
        GpioMiso::mode(Mode::ALTERNATE);
        GpioMiso::alternateFunction(GetAlternativeFunctionNumber(N));
        GpioSclk::mode(Mode::ALTERNATE);
        GpioSclk::alternateFunction(GetAlternativeFunctionNumber(N));
        usleep(CS_DELAY);
        EnablePeriphBus(GetSpiAddr(N));
        GetSpiAddr(N)->CR1 = SPI_CR1_SSM
                           | SPI_CR1_SSI
                           | SPI_CR1_MSTR
                           | SPI_CR1_BR_2
                           | SPI_CR1_SPE;
    }

    inline static constexpr int GetAlternativeFunctionNumber(int n_spi) {
        return n_spi == 1 || n_spi == 2 ? 5 : 6;
    }

    constexpr SPI_TypeDef* GetSpiAddr(unsigned n) {
        return  n==1 ? SPI1 :
                n==2 ? SPI2 : SPI3;
    }

    static inline void EnablePeriphBus(SPI_TypeDef* spi) {
        if(spi == SPI1)
            RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
        else if(spi == SPI2)
            RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
        else if(spi == SPI3)
            RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    }
};

template<unsigned N>
class BusI2C {
    friend class Singleton< BusI2C<N> >;
public:
    static void Init() { 
        Singleton< BusI2C<N> >::GetInstance(); 
    }
    static int Write(const void* buffer, size_t len) {
        return Singleton<BusI2C<N>>::GetInstance()->_write(buffer, len);
    }
    static int Read(void* buffer, size_t max_len) {
        return Singleton<BusI2C<N>>::GetInstance()->_read(buffer, max_len);
    }
protected:
    int _write(const void* buffer, size_t len) {
        // DMA??
        return 0;
    }
    int _read(void* buffer, size_t max_len) {
        // conditional var??
        return 0;
    }
    inline BusI2C() { }
};

template<class Bus, class GpioCS>
class ProtocolSPI {
public:
    static void Init() {
        GpioCS::mode(Mode::OUTPUT);
        GpioCS::high();
        Bus::Init();
    }

    static uint8_t ReadReg(uint8_t reg) {
        GpioCS::low();
        reg |= 0x80;
        Bus::Write(&reg, sizeof(reg));
        Bus::Read(&reg, sizeof(reg));
        GpioCS::high();
        return reg;
    }

    static void WriteReg(uint8_t reg, uint8_t val) {
        GpioCS::low();
        Bus::Write(&reg, sizeof(reg));
        Bus::Write(&val, sizeof(reg));
        GpioCS::high();
    }
};

template<class Bus, unsigned ID>
class ProtocolI2C {
public:
    static inline void Init() {
        Bus::Init();\
    }

    static uint8_t ReadReg(uint8_t reg)
    {
        static const uint8_t id = ID;
        Bus::Write(&id, sizeof(id));
        Bus::Write(&reg, sizeof(reg));
        Bus::Read(&reg, sizeof(reg));
        return reg;
    }

};

template<class Protocol>
class Sensor {
public:
    inline Sensor() { 
        Protocol::Init(); 
    }
    inline static uint8_t ReadReg(uint8_t reg) { 
        return Protocol::ReadReg(reg); 
    }
    inline static void WriteReg(uint8_t reg, uint8_t value) { 
        Protocol::WriteReg(reg, value); 
    }
    virtual void Init() = 0;
    virtual bool Test() const = 0;
};

class RegMap_AXEL {
public:
    static constexpr uint8_t REG_X_L        = 0x28;
    static constexpr uint8_t REG_X_H        = 0x29;
    static constexpr uint8_t REG_Y_L        = 0x2A;
    static constexpr uint8_t REG_Y_H        = 0x2B;
    static constexpr uint8_t REG_Z_H        = 0x2C;
    static constexpr uint8_t REG_Z_L        = 0x2D;
    static constexpr uint8_t REG_STAT       = 0x2E;
    static constexpr uint8_t REG_WHO_AM_I   = 0x0F;
};

template<class RegMap>
class Axel : public Sensor<ProtocolI2C<BusI2C<2>, 3>> {
    // assert bus class here
public:
    int ReadAxelX() {
        return Sensor::ReadReg(RegMap::REG_X_L);
    }
    int ReadWhoAmI() {
        return Sensor::ReadReg(RegMap::REG_WHO_AM_I);
    }
};

#endif // BUSTEMPLATE_H
