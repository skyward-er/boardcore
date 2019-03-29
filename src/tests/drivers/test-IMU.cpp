#include <sensors/LSM6DS3H.h>

// SPI1
typedef miosix::Gpio<GPIOA_BASE, 5> GpioSck;
typedef miosix::Gpio<GPIOA_BASE, 6> GpioMiso;
typedef miosix::Gpio<GPIOA_BASE, 7> GpioMosi;
typedef miosix::Gpio<GPIOC_BASE, 3> CS_MPU9250;

// SPI1 binding al sensore
typedef BusSPI<1, GpioMosi, GpioMiso, GpioSck> busSPI1;  // Creo la SPI1
typedef ProtocolSPI<busSPI1, CS_MPU9250>
    spiProt;                      // La lego al Chip Select 1 per la IMU 1
typedef LSM6DS3H<spiProt> lsm_t;  // Passo il bus creato al sensore

int main (){
    lsm_t* lsm = new  lsm_t(16, 500);
    UNUSED(lsm);
    while(1)
    {

    }
}