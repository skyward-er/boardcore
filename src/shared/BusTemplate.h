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
        return SingletonType::GetInstance()->_write(buffer, len);
    }
    inline static int write(uint8_t byte) {
        return SingletonType::GetInstance()->_write(byte);
    }

    inline static void init() { 
        SingletonType::GetInstance(); 
    }
    inline static int read(void* buffer, size_t max_len) {
        return SingletonType::GetInstance()->_read(buffer, max_len);
    }

    inline static uint8_t read() {
        return SingletonType::GetInstance()->_read();
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

    static uint8_t read(uint8_t reg) {
        GpioCS::low();
        reg |= 0x80;
        Bus::write(&reg, sizeof(reg));
        Bus::read(&reg, sizeof(reg));
        GpioCS::high();
        return reg;
    }

    static void write(uint8_t reg, uint8_t val) {
        GpioCS::low();
        Bus::write(&reg, sizeof(reg));
        Bus::write(&val, sizeof(reg));
        GpioCS::high();
    }
private:
    ProtocolSPI() = delete;
    ~ProtocolSPI() = delete;
    ProtocolSPI(const ProtocolSPI& o) = delete;
    ProtocolSPI(const ProtocolSPI&& o) = delete;
    ProtocolSPI& operator=(const ProtocolSPI& other);
};

template<class Bus, unsigned ID>
class ProtocolI2C {
    public:
        static inline void init() {
            bus.init(); // FIXME: @redman: what happens if this is called multiple times?
        }
        
        /**
         * Sends the \param len bytes stored in \param *data buffer to the register specified
         * by \param regAddress        
         */
        static void write(uint8_t addr, uint8_t *data, uint8_t len) {
            uint8_t buf[len+1];     //pack register address and payload
            buf[0] = addr;
            
            memcpy(buf+1, data, len);
            
            bus.send(ID, reinterpret_cast<void*>(buf), len+1);
        }
        
        /**
         * Reads \param len bytes storing them into \param *data buffer from the register specified
         * by \param regAddress        
         */
        static void read(uint8_t addr, uint8_t *data, uint8_t len) {
            bus.send(ID,reinterpret_cast<void*>(&addr),1);
            bus.receive(ID,reinterpret_cast<void*>(data),len);
        }
        
    private:
        static Bus &bus = Bus::instance();

        ProtocolI2C() = delete;
        ~ProtocolI2C() = delete;
        ProtocolI2C(const ProtocolI2C& o) = delete;
        ProtocolI2C(const ProtocolI2C&& o) = delete;
        ProtocolI2C& operator=(const ProtocolI2C& other);
};

#endif // BUSTEMPLATE_H
