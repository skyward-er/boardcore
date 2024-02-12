#include "qspi-flash.h"

using namespace miosix;
using namespace Boardcore;
using namespace FlashMemory;


/**
 * QSPI Flash pins
 *
 * FLASH_NSS - PB10 - AF9  - QUADSPI_BK1_NCS
 * FLASH_CLK - PF10 - AF9  - QUADSPI_CLK
 * FLASH_IO0 - PF8  - AF10 - QUADSPI_BK1_IO0
 * FLASH_IO1 - PF9  - AF10 - QUADSPI_BK1_IO1
 * FLASH_IO2 - PF7  - AF9  - QUADSPI_BK1_IO2
 * FLASH_IO3 - PF6  - AF9  - QUADSPI_BK1_IO3
 */

GpioPin flash_ncs(GPIOB_BASE, 10);
GpioPin flash_sck(GPIOF_BASE, 10);
GpioPin flash_io0(GPIOF_BASE, 8);
GpioPin flash_io1(GPIOF_BASE, 9);
GpioPin flash_io2(GPIOF_BASE, 7);
GpioPin flash_io3(GPIOF_BASE, 6);


void QSPI::enable() { QUADSPI->CR |= QUADSPI_CR_EN; }


void QSPI::disable() { QUADSPI->CR &= ~QUADSPI_CR_EN; }


void QSPI::init() {

    // init GPIO peripheral pins
    flash_ncs.mode(Mode::ALTERNATE);
    flash_ncs.alternateFunction(9);
    flash_ncs.speed(Speed::_100MHz);
    flash_sck.mode(Mode::ALTERNATE);
    flash_sck.alternateFunction(9);
    flash_sck.speed(Speed::_100MHz);
    flash_io0.mode(Mode::ALTERNATE);
    flash_io0.alternateFunction(10);
    flash_io0.speed(Speed::_100MHz);
    flash_io1.mode(Mode::ALTERNATE);
    flash_io1.alternateFunction(10);
    flash_io1.speed(Speed::_100MHz);
    flash_io2.mode(Mode::ALTERNATE);
    flash_io2.alternateFunction(9);
    flash_io2.speed(Speed::_100MHz);
    flash_io3.mode(Mode::ALTERNATE);
    flash_io3.alternateFunction(9);
    flash_io3.speed(Speed::_100MHz);

    // init peripheral clock 
    ClockUtils::enablePeripheralClock((QUADSPI_TypeDef*)QSPI_BASE);

    RCC_SYNC();

    Thread::sleep(200); 

    // abort possible ongoing command 
    QUADSPI->CR |= QUADSPI_CR_ABORT;
    
    // Wait while aborted
    while (QUADSPI->CR & QUADSPI_CR_ABORT) {;}  
    
    // disable peripheral 
    QSPI::disable(); 

    // reset configuration registers 
    QUADSPI->CR  = 0;
    QUADSPI->DCR = 0;
    QUADSPI->CCR = 0;
    QUADSPI->DLR = 0;
    
    // reset transfer complete flag (TCF)
    QUADSPI->FCR &= ~(1 << QUADSPI_FCR_CTCF_Pos); 

    // peripheral default initialization
    QUADSPI->CR |= QUADSPI_CR_SSHIFT |  // Wait a full cycle to read
        3 << QUADSPI_CR_PRESCALER_Pos;  // QSPI clock = 216MHz / 4 = 54MHz
    // QUADSPI->DCR |=
    //     21 << QUADSPI_DCR_FSIZE_Pos;  // Flash size 32Mb = 4MB = 2^(21+1)
    //     bytes

}


void QSPI::waitBusy() {
    while(!(QUADSPI->SR & (1 << QUADSPI_SR_BUSY_Pos))) {;}
}


void QSPI::waitTransfer() {

    // wait till the end of the communication
    while (!(QUADSPI->SR & (1 << QUADSPI_SR_TCF_Pos))) {;}

    // reset transfer complete flag (TCF)
    QUADSPI->FCR &= ~(1 << QUADSPI_FCR_CTCF_Pos); 

}


// constructor class qspi_flash
qspi_flash::qspi_flash() {;}


uint8_t qspi_flash::read_status_reg() {
     
    // indirect read mode, la comunicazione comincia dopo aver scritto l'istruzione 
    // nel CCR register. 
    // 1 invio Read status register command
    // ricevo un byte di dati con il valore dello status register. 

    QSPI::disable(); 

    QSPI::init();  

    QUADSPI->CCR |= 1 << QUADSPI_CCR_FMODE_Pos |   // Indirect read mode
                        1 << QUADSPI_CCR_DMODE_Pos |   // Data on 1-wire
                        0 << QUADSPI_CCR_ABMODE_Pos |  // No alternate bytes
                        0 << QUADSPI_CCR_ADMODE_Pos |  // No address
                        1 << QUADSPI_CCR_IMODE_Pos;    // Instruction on 1-wire
    
    // Expect to receive 1 byte (flash status register)
    QUADSPI->DLR = 0;   

    // DEVE ESSERE FATTO PRIMA DI SCRIVERE L'ISTRUZIONE NEL REGISTRO, 
    // PERCHè IN QUESTO CASO LA COMUNICAZIONE COMINCIA SCIVENDO L'ISTRUZIONE IN CCR 
    QSPI::enable();
    
    // Trigger communication start by writing the instruction
    QUADSPI->CCR |= Commands::READ_STATUS_REG << QUADSPI_CCR_INSTRUCTION_Pos; 

    // wait till data trasfer is complete
    QSPI::waitTransfer();

    return QUADSPI->DR; 
}


void qspi_flash::write_enable() {
    
    // in indirect write mode with no data or address phase (DMODE = 0, ADMODE = 0) the 
    // communication starts whenever the CCR register is written. 
    // 1 send wren command
    // 2 read status register 
    // 3 wait for bit WEL = 1
    // poi program/erase commands 
    
    QSPI::disable(); 
    
    QSPI::init(); // reset previous configurations and init clock and gpio

    // indirect write mode, istruction on 1 wire, no data, no address, no alternate bytes 
    QUADSPI->CCR |= 1 << QUADSPI_CCR_IMODE_Pos; 

    QSPI::enable(); 

    // start communication writing the instruction to CCR register busy bit = 1
    QUADSPI->CCR |= Commands::WRITE_ENABLE; 

    QSPI::waitBusy(); 

    QSPI::disable();

    printf("prima del whileeee\n"); 

    // read status register until WEL is set (write enable latch)
    uint8_t status = read_status_reg(); 
    do {
        Thread::sleep(1); 
        status = read_status_reg();
    }
    while(!(status & 1 << 1)); // WEL_pos = 1 è il secondo bit
}



uint32_t qspi_flash::readID() {

    QSPI::disable(); 

    QSPI::init(); 

    QUADSPI->CCR |= 1 << QUADSPI_CCR_FMODE_Pos |   // Indirect read mode
                        1 << QUADSPI_CCR_DMODE_Pos |   // Data on 1-wire
                        0 << QUADSPI_CCR_ABMODE_Pos |  // No alternate bytes
                        0 << QUADSPI_CCR_ADMODE_Pos |  // No address
                        1 << QUADSPI_CCR_IMODE_Pos;    // Instruction on 1-wire
    
    // Expect to receive 3 bytes regarding ID of the flash
    QUADSPI->DLR = 2;   

    // DEVE ESSERE FATTO PRIMA DI SCRIVERE L'ISTRUZIONE NEL REGISTRO, 
    // PERCHè IN QUESTO CASO LA COMUNICAZIONE COMINCIA SCIVENDO L'ISTRUZIONE IN CCR 
    QSPI::enable();
    
    // Trigger communication start by writing the instruction
    QUADSPI->CCR |= Commands::READ_ID << QUADSPI_CCR_INSTRUCTION_Pos; 

    // wait till communication is ended
    QSPI::waitTransfer(); 

    // if there are some bytes in the quadspi buffer (FIFO), read them
    if (QUADSPI->SR & (63 << QUADSPI_SR_FLEVEL_Pos)) {
        uint32_t myID = QUADSPI->DR; 
        QSPI::disable();
        return myID;
    }
    else {
        QSPI::disable();
        return -1; 
    }
}