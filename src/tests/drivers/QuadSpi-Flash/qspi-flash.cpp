/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Valerio Flamminii
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

void QSPI::init()
{

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

    Thread::sleep(1);

    // abort possible ongoing command
    QUADSPI->CR |= QUADSPI_CR_ABORT;

    // Wait while aborted
    while (QUADSPI->CR & QUADSPI_CR_ABORT)
    {
        ;
    }

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
    QUADSPI->CR |=
        QUADSPI_CR_SSHIFT |             // Wait a full cycle to read
        3 << QUADSPI_CR_PRESCALER_Pos;  // QSPI clock = 216MHz / 4 = 54MHz

    // -------------- must be setted for read function ----------------------
    // impostare la dimensione della memoria è necessario alle operazioni di
    // lettura della flash
    QUADSPI->DCR |=
        21 << QUADSPI_DCR_FSIZE_Pos;  // Flash size 32Mb = 4MB = 2^(21+1) bytes
}

void QSPI::abort_reset()
{

    // abort possible ongoing command
    QUADSPI->CR |= QUADSPI_CR_ABORT;

    // Wait while aborted
    while (QUADSPI->CR & QUADSPI_CR_ABORT)
    {
        ;
    }

    // to be sure that the peripheral is disabled
    QSPI::disable();

    // reset configuration register
    QUADSPI->CCR = 0;

    // transfer flag (TCF) will reset automatically and FIFO is flushed
    // if a transaction has been aborted.
}

void QSPI::waitBusy()
{
    while (QUADSPI->SR & (1 << QUADSPI_SR_BUSY_Pos))
    {
        ;
    }
}

void QSPI::waitTransfer()
{

    // wait till the end of the communication
    while (!(QUADSPI->SR & (1 << QUADSPI_SR_TCF_Pos)))
    {
        ;
    }

    // reset transfer complete flag (TCF)
    QUADSPI->FCR &= ~(1 << QUADSPI_FCR_CTCF_Pos);
}

// constructor class qspi_flash
qspi_flash::qspi_flash() { ; }

bool qspi_flash::test()
{
    // init qspi peripheral and check the device ID of the flash

    QSPI::init();
    return readID() == DEVICE_ID ? true : false;
}

uint8_t qspi_flash::read_status_reg()
{

    // status register can be read at any time and during every kind of
    // operation. indirect read mode, la comunicazione comincia dopo aver
    // scritto l'istruzione nel CCR register. 1 invio Read status register
    // command 2 ricevo un byte di dati con il valore dello status register.

    QSPI::abort_reset();

    QUADSPI->CCR |= 1 << QUADSPI_CCR_FMODE_Pos |   // Indirect read mode
                    1 << QUADSPI_CCR_DMODE_Pos |   // Data on 1-wire
                    0 << QUADSPI_CCR_ABMODE_Pos |  // No alternate bytes
                    0 << QUADSPI_CCR_ADMODE_Pos |  // No address
                    1 << QUADSPI_CCR_IMODE_Pos;    // Instruction on 1-wire

    // Expect to receive 1 byte (flash status register)
    QUADSPI->DLR = 0;

    // DEVE ESSERE FATTO PRIMA DI SCRIVERE L'ISTRUZIONE NEL REGISTRO,
    // PERCHè IN QUESTO CASO LA COMUNICAZIONE COMINCIA SCIVENDO L'ISTRUZIONE IN
    // CCR
    QSPI::enable();

    // Trigger communication start by writing the instruction
    QUADSPI->CCR |= Commands::READ_STATUS_REG << QUADSPI_CCR_INSTRUCTION_Pos;

    // wait till data trasfer is complete
    QSPI::waitTransfer();

    // dopo waitTranfer() mi aspetto ci sia solo un byte nella fifo, posso fare
    // direttamente il cast a uint8_t (byte)
    uint32_t value = (uint8_t)QUADSPI->DR;

    // disable peripheral
    QSPI::disable();

    return value;
}

void qspi_flash::write_enable()
{

    // in indirect write mode with no data or address phase (DMODE = 0, ADMODE =
    // 0) the communication starts whenever the CCR register is written. 1 send
    // wren command 2 read status register 3 wait for bit WEL = 1 poi
    // program/erase commands

    QSPI::abort_reset();

    // indirect write mode, istruction on 1 wire, no data, no address, no
    // alternate bytes
    QUADSPI->CCR |= 1 << QUADSPI_CCR_IMODE_Pos;

    QSPI::enable();

    // start communication writing the instruction to CCR register busy bit = 1
    QUADSPI->CCR |= Commands::WRITE_ENABLE;

    QSPI::waitBusy();

    QSPI::disable();

    // check status register until bit WEL = 1
    uint8_t status = read_status_reg();

    while (!(status & (1 << 1)))
    {  // WEL_pos = 1 è il secondo bit
        Thread::sleep(1);
        status = read_status_reg();
    }
}

void qspi_flash::init() { QSPI::init(); }

uint32_t qspi_flash::readID()
{

    QSPI::abort_reset();

    QUADSPI->CCR |= 1 << QUADSPI_CCR_FMODE_Pos |   // Indirect read mode
                    1 << QUADSPI_CCR_DMODE_Pos |   // Data on 1-wire
                    0 << QUADSPI_CCR_ABMODE_Pos |  // No alternate bytes
                    0 << QUADSPI_CCR_ADMODE_Pos |  // No address
                    1 << QUADSPI_CCR_IMODE_Pos;    // Instruction on 1-wire

    // Expect to receive 3 bytes regarding ID of the flash
    QUADSPI->DLR = 2;

    QSPI::enable();

    // Trigger communication by writing the instruction into CCR register
    QUADSPI->CCR |= Commands::READ_ID << QUADSPI_CCR_INSTRUCTION_Pos;

    // wait till communication is ended
    QSPI::waitTransfer();

    // sus: even if some bytes have been received, the FIFO need some time to
    // store them.
    Thread::sleep(1);

    // if there are some bytes in the quadspi buffer (FIFO), read them
    if ((QUADSPI->SR & QUADSPI_SR_FLEVEL) >> 8)
    {
        uint32_t myID = QUADSPI->DR;

        QSPI::disable();

        // the ID bytes order must be inverted
        uint8_t lb = myID & (255U);          // lowest byte
        uint8_t mb = (myID >> 8) & (255U);   // middle byte
        uint8_t hb = (myID >> 16) & (255U);  // highest byte

        myID = (lb << 16) | (mb << 8) | (hb);

        return myID;
    }
    else
    {
        QSPI::disable();
        return 0;
    }
}

void qspi_flash::write_disable()
{

    // in indirect write mode with no data or address phase (DMODE = 0, ADMODE =
    // 0) the communication starts whenever the CCR register is written. 1 send
    // wrid command (0x4) 2 read status register 3 wait for bit WEL = 0 poi
    // return

    QSPI::abort_reset();

    // indirect write mode, istruction on 1 wire, no data, no address, no
    // alternate bytes
    QUADSPI->CCR |= 1 << QUADSPI_CCR_IMODE_Pos;

    QSPI::enable();

    // start communication writing the instruction to CCR register busy bit = 1
    QUADSPI->CCR |= Commands::WRITE_DISABLE;

    QSPI::waitBusy();

    QSPI::disable();

    // check status register until bit WEL = 0
    uint8_t status = read_status_reg();

    while (status & (1 << 1))
    {  // WEL è il secondo bit
        Thread::sleep(1);
        status = read_status_reg();
    }
}

bool qspi_flash::isInProgress()
{

    // check if the memory is executing some operation.
    // bit WIP in flash status register is set if a program/erase/write to
    // registers is being executed.
    // 1 read flash status register
    // 2 bit WIP = 1: operation in progress / WIP = 0 no one operation in
    // progress.

    uint8_t status_reg = read_status_reg();
    return (status_reg & 1) ? true : false;
}

void qspi_flash::waitProgress()
{

    while (isInProgress())
    {
        Thread::sleep(1);
    }
}

uint8_t qspi_flash::read_byte(uint32_t address)
{

    // THE MEMORY SIZE NEED TO BE SETTED IN DCR REGISTER !!!!!!!

    // read one byte from the memory starting at a specific address.
    // 1 send READ command
    // 2 send 3-byte address
    // 3 read only the first byte of data
    // in this case, since no data is needed, the communication starts whenever
    // the adress register (QUADSPI->DR) is updated.

    // check on correct address range
    if (address > FlashMemory::MEMORY_SIZE)
        return 0;

    QSPI::abort_reset();

    QUADSPI->CCR |= 1 << QUADSPI_CCR_FMODE_Pos |  // Indirect read mode
                    QUADSPI_CCR_DMODE_0 |         // data on a single  line
                    QUADSPI_CCR_ADSIZE_1 |        // 3-byte (24 bit) address
                    QUADSPI_CCR_ADMODE_0 |        // address on a single line
                    QUADSPI_CCR_IMODE_0;  // instruction on a single line

    // read istruction in CCR register
    QUADSPI->CCR |= Commands::READ;

    // just 1 byte of data is supposed to be transferred
    QUADSPI->DLR = 0;

    QSPI::enable();

    // start communication by specifing the address
    QUADSPI->AR = address;

    QSPI::waitTransfer();

    uint8_t value = (uint8_t)QUADSPI->DR;

    QSPI::disable();

    return value;
}

bool qspi_flash::chip_erase()
{

    // erase the entire flash memory chip
    // erase chip operation typical time: 30-60 s
    // 1 wait until the memory has finished any operation in progress
    // 2 write_enable command
    // 3 erase chip command
    // 4 wait till flash has completed the erase operation
    // 5 write_disable command to be sure
    // 6 read flag E_FAIL in security register to ckeck that the last
    // erase operation has succeded.
    // return: true = erase chip operation succeded, false = erase chip
    // operation failed

    waitProgress();

    write_enable();

    QSPI::abort_reset();

    // set quadspi CCR register:
    // indirect write mode, no address, no data. all on one wire.
    QUADSPI->CCR |= QUADSPI_CCR_IMODE_0;  // istruction on one wire

    QSPI::enable();

    // write istruction and start the communication
    QUADSPI->CCR |= Commands::ERASE_CHIP;

    QSPI::waitBusy();

    QSPI::disable();

    // wait till end of erase operation
    waitProgress();

    write_disable();

    // return the result of chip erase operation
    return check_erase();
}

bool qspi_flash::sector_erase(uint32_t address)
{

    // erase a specific sector (4 KB), any address of the sector is a valid
    // address. 1 wait until the memory has finished any operation in progress
    // 2 write_enable command
    // 3 erase sector command
    // 4 wait till flash has completed the operation
    // 5 write_disable command to be sure
    // 6 check the result (flasg E_FAIL)

    // check on address range
    if ((address < 0) || (address > FlashMemory::MEMORY_SIZE))
        return false;

    waitProgress();

    write_enable();

    QSPI::abort_reset();

    // set quadspi CCR register:
    // indirect write mode, 3-byte address , no data. all on one wire.
    QUADSPI->CCR |= QUADSPI_CCR_IMODE_0 |   // istruction on one wire
                    QUADSPI_CCR_ADSIZE_1 |  // 3-byte address
                    QUADSPI_CCR_ADMODE_0;   // address on a single line

    QSPI::enable();

    // add instruction
    QUADSPI->CCR |= Commands::SECTOR_ERASE;

    // start communication by writing the address in QUADSPI->AR
    QUADSPI->AR = address;

    QSPI::waitBusy();

    QSPI::disable();

    waitProgress();

    write_disable();

    return check_erase();
}

bool qspi_flash::block32_erase(uint32_t address)
{

    // erase a 32K block of data, any address of the block is valid
    // 1 wait until the memory has finished any operation in progress
    // 2 write_enable command
    // 3 erase block_32 command
    // 4 wait till flash has completed the operation
    // 5 write_disable command to be sure
    // 6 check the result (flasg E_FAIL)

    // check on correct address range
    if ((address < 0) || (address > FlashMemory::MEMORY_SIZE))
        return false;

    waitProgress();

    write_enable();

    QSPI::abort_reset();

    // set quadspi CCR register
    // indirect write mode, 3-byte address, no data. all on one wire.
    QUADSPI->CCR |= QUADSPI_CCR_IMODE_0 |   // istruction on one wire
                    QUADSPI_CCR_ADSIZE_1 |  // 3-byte address
                    QUADSPI_CCR_ADMODE_0;   // address on a single line

    QSPI::enable();

    // add instruction
    QUADSPI->CCR |= Commands::BLOCK_32_ERASE;

    // start communication by writing the address in QUADSPI->AR
    QUADSPI->AR = address;

    QSPI::waitBusy();

    QSPI::disable();

    waitProgress();

    write_disable();

    return check_erase();
}

bool qspi_flash::block64_erase(uint32_t address)
{

    // erase a 64K block of data, any address of the block is valid
    // 1 wait until the memory has finished any operation in progress
    // 2 write_enable command
    // 3 erase block_64 command
    // 4 wait till flash has completed the operation
    // 5 write_disable command, just to be sure
    // 6 check the result (flasg E_FAIL)

    // check on correct address range
    if ((address < 0) || (address > FlashMemory::MEMORY_SIZE))
        return false;

    waitProgress();

    write_enable();

    QSPI::abort_reset();

    // set quadspi CCR register
    // indirect write mode, 3-byte address, no data. all on one wire.
    QUADSPI->CCR |= QUADSPI_CCR_IMODE_0 |   // istruction on one wire
                    QUADSPI_CCR_ADSIZE_1 |  // 3-byte address
                    QUADSPI_CCR_ADMODE_0;   // address on a single line

    QSPI::enable();

    // add instruction
    QUADSPI->CCR |= Commands::BLOCK_64_ERASE;

    // start communication by writing the address in QUADSPI->AR
    QUADSPI->AR = address;

    QSPI::waitBusy();

    QSPI::disable();

    waitProgress();

    write_disable();

    return check_erase();
}

bool qspi_flash::byte_program(uint8_t data, uint32_t address, bool verify)
{

    // double-check data may take some extra time !

    // write a byte at the specified address (any address is ok)
    // 1 wait till every memory operation is ended
    // 2 write enable
    // 3 page program command
    // 4 wait till the operation is ended
    // 5 write disable
    // 6 return the result

    // check on address range
    if ((address < 0) || (address > FlashMemory::MEMORY_SIZE))
        return false;

    waitProgress();

    write_enable();

    QSPI::abort_reset();

    // idirect write mode
    QUADSPI->CCR |= QUADSPI_CCR_DMODE_0 |   // data on a single line
                    QUADSPI_CCR_ADSIZE_1 |  // address size 24 bit
                    QUADSPI_CCR_ADMODE_0 |  // address on a single line
                    QUADSPI_CCR_IMODE_0;    // instruction on a single line

    QSPI::enable();

    // page program command
    QUADSPI->CCR |= Commands::PAGE_PROGRAM;

    // address
    QUADSPI->AR = address;

    // trigger the communication by writing into data register
    QUADSPI->DR = data;

    QSPI::waitBusy();

    waitProgress();

    write_disable();

    // if verify = true, double check on byte program operation
    if (verify == true)
    {
        if (read_byte(address) != data)
            return false;
    }

    return check_program();
}

uint8_t qspi_flash::read_security_reg()
{

    // security register can be read at any time and during every kind of
    // operation. in indirect read mode: 1 - send read security register command
    // (1-byte instruction) 2 - receive one byte of data containing the value of
    // the register.

    QSPI::abort_reset();

    QUADSPI->CCR |= QUADSPI_CCR_FMODE_0 |  // Indirect read mode
                    QUADSPI_CCR_DMODE_0 |  // data on one wire
                    QUADSPI_CCR_IMODE_0;   // instruction on one wire

    // Expect to receive 1 byte (flash security register value)
    QUADSPI->DLR = 0;

    QSPI::enable();

    // start communication by writing the instruction
    QUADSPI->CCR |= Commands::READ_SECURITY_REG;

    // wait till data trasfer is complete
    QSPI::waitTransfer();

    // sus, ma funziona, stesso problema nella readID()
    Thread::sleep(1);

    uint32_t value = (uint8_t)QUADSPI->DR;

    QSPI::disable();

    return value;
}

void qspi_flash::software_reset()
{

    // if software reset is performed at the same time of another program/erase
    // operation, that current operation will not be executed correctly, and
    // some data could be lost.

    // this functionality combines two commands, RSTEN and RST,
    // they need to be sent according to this order:
    // 1 - RSTEN instruction (reset enable)
    // 2 - RST   instruction (software reset)
    // if any operation is executed before the RST instruction, the reset enable
    // (RSTEN) is invalid.

    // commands sequence: RSTEN>>wait 1ms>>RST>>wait 1ms
    // -------------------- send RSTEN command -------------------------
    waitProgress();

    QSPI::abort_reset();

    // indirect write mode, no data, no address, instruction on a single line
    QUADSPI->CCR |= QUADSPI_CCR_IMODE_0;

    QSPI::enable();

    // start the communication by writing the reset enable instruction
    QUADSPI->CCR |= Commands::RESET_ENABLE;

    QSPI::waitBusy();

    QSPI::disable();

    Thread::sleep(1);  // wait 1 ms

    // ------------------- send RST command --------------------------
    QSPI::abort_reset();

    QUADSPI->CCR |= QUADSPI_CCR_IMODE_0;

    QSPI::enable();

    QUADSPI->CCR |= Commands::RESET_MEMORY;

    QSPI::waitBusy();

    QSPI::disable();

    Thread::sleep(1);
}

bool qspi_flash::check_erase()
{

    // ATTTENTION! - this function check only if the last erase operation has
    // been completed, so it's not so reliable ! check for bit E_FAIL in
    // security register of memory: E_fail = 1 the last erase operation has
    // failed e_fail = 0 tha last operation has succeded returns true  - erase
    // operation has succeded returns false - erase operation has failed

    uint8_t reg = read_security_reg();
    return reg & (1 << 6) ? false : true;
}

bool qspi_flash::check_program()
{

    // ATTTENTION! - this function check only if the last operation has been
    // completed, so it's not so reliable ! check for bit P_FAIL in security
    // register of memory: returns true  - erase operation has succeded returns
    // false - erase operation has failed

    uint8_t reg = read_security_reg();
    return reg & (1 << 5) ? false : true;
}

bool qspi_flash::read_sector(std::vector<uint8_t>& vector, uint32_t sector_num)
{

    // read an entire sector of the flash into a vector
    // that will modify the vector and his elements.
    // that fix size and capacity of the vector to SECTOR_SIZE.

    if (sector_num < 0 || sector_num > SECTORS_NUM)
    {
        return false;
    }

    // reserve enough bytes to retain a sector of the flash, also
    // make sure that size parameter match with the capacity.
    if (vector.capacity() < SECTOR_SIZE)
    {
        vector.reserve(SECTOR_SIZE);
        vector.resize(vector.capacity());  // SUS, altrimenti size != capacity
    }

    if (vector.capacity() >= SECTOR_SIZE)
    {
        vector.resize(SECTOR_SIZE);
        vector.shrink_to_fit();
    }

    uint32_t addr = SECTOR_SIZE * sector_num;

    uint32_t index = 0;
    for (index = 0; index < vector.capacity(); index++)
    {
        vector[index] = read_byte(addr);
        addr++;
    }

    return true;
}

bool qspi_flash::page_program(std::vector<uint8_t>& vector,
                              uint32_t start_address, bool verify)
{

    // fifo interna: 32 bytes
    // imposta prima il dlr
    // scrivi byte per byte nel DR
    // program the last 256 bytes of a vector starting by a specific page
    // address. WEL bit is set to zero automatically after the program
    // operation. NO WRITE_DISABLE the address specified must be a starting
    // address of a page !!!!

    // not a starting address of a page
    if ((start_address % PAGE_SIZE) != 0)
        return false;

    // start_address out of address space of the memory
    if (start_address >= MEMORY_SIZE)
        return false;

    // empty vector
    if (vector.size() == 0)
        return false;

    // vector bigger than a page size
    if (vector.size() > PAGE_SIZE)
        return false;

    write_enable();

    QSPI::abort_reset();

    // idirect write mode
    QUADSPI->CCR |= QUADSPI_CCR_DMODE_0 |   // data on a single line
                    QUADSPI_CCR_ADSIZE_1 |  // address size 24 bit
                    QUADSPI_CCR_ADMODE_0 |  // address on a single line
                    QUADSPI_CCR_IMODE_0;    // instruction on a single line

    QSPI::enable();

    // page program command
    QUADSPI->CCR |= Commands::PAGE_PROGRAM;

    // number of bytes to be transferred
    QUADSPI->DLR = vector.size() - 1;

    // starting address
    QUADSPI->AR = start_address;

    // load data vector into the QUADSPI FIFO
    uint16_t i = 0;
    for (i = 0; i < vector.size(); i++)
    {

        // if FIFO is full - wait till it has at least a byte available.
        while (((QUADSPI->SR & QUADSPI_SR_FLEVEL) >> 8) >= 32)
        {
            ;
        }

        // add a single byte to be sent into the QSPI FIFO
        ((uint8_t*)&QUADSPI->DR)[0] = static_cast<uint8_t>(vector[i]);
    }

    QSPI::waitBusy();

    waitProgress();

    // if verify flag is set, double check on written data
    if (verify == true)
    {
        for (i = 0; i < vector.size(); i++)
        {

            if (read_byte(start_address + i) != vector[i])
                return false;
        }
    }

    return check_program();
}

bool qspi_flash::write_vector(std::vector<uint8_t>& vector, uint32_t sector_num,
                              bool verify_write)
{

    // this function program into the flash just the real elements of the vector
    // (size). THE VECTOR PASSED MUST HAVE SAME SIZE AND CAPACITY. return true:
    // program operation succeded return false: program operation failed

    // wrong secton_num specified
    if (sector_num < 0 || sector_num >= SECTORS_NUM)
    {
        return false;
    }

    // if the vector doesn't have elements
    if (vector.size() == 0 || vector.capacity() == 0)
    {
        return false;
    }

    // if the vector is bigger than the flash capacity
    if (vector.size() > MEMORY_SIZE)
        return false;

    // if the whole vector is bigger than the rest of sectors starting by
    // sector_num
    if (vector.size() > (SECTOR_SIZE * (SECTORS_NUM - sector_num)))
        return false;

    // constant start address
    uint32_t const start_address = SECTOR_SIZE * sector_num;

    // compute number of sectors needed to store the vector, then add one more
    // partial sector for the last part of vector.
    uint32_t sectors_needed = vector.size() / SECTOR_SIZE;
    if ((vector.size() % SECTOR_SIZE) != 0)
        sectors_needed += 1;

    // erase all sectors needed to store the entire vector
    uint32_t sector_index = 0;
    for (sector_index = 0; sector_index < sectors_needed; sector_index++)
    {
        if (sector_erase(SECTOR_SIZE * (sector_num + sector_index)) == false)
            return false;
    }

    // compute the number of pages needed to store the entire vector
    uint32_t pages_needed = vector.size() / PAGE_SIZE;
    if ((vector.size() % PAGE_SIZE) != 0)
        pages_needed += 1;

    // creating a copy vector
    std::vector<uint8_t> v;
    v.reserve(PAGE_SIZE);
    v.resize(0);

    // for every page needed
    uint32_t page = 0;
    uint32_t elem = 0;
    for (page = 0; page < pages_needed; page++)
    {
        v.resize(0);
        uint32_t start_elem = page * PAGE_SIZE;
        for (elem = start_elem;
             (elem < start_elem + PAGE_SIZE) && (elem < vector.size()); elem++)
        {
            v.push_back(vector[elem]);
        }

        if (page_program(v, start_address + (page * PAGE_SIZE), false) == false)
            return false;
    }

    // if verify_write is true:
    // check that the bytes written into the flash match with the ones in the
    // vector.
    if (verify_write)
    {

        uint32_t index = 0;
        uint32_t addr  = start_address;
        for (index = 0; index < vector.size(); index++)
        {
            if (read_byte(addr) != vector[index])
            {
                return false;
            }
            addr++;
        }
    }

    return true;
}