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

    // set the memory chip size - it must be always setted before any read
    // operation.
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

    // transfer flag (TCF) will reset automatically and QUADSPI FIFO is flushed
    // if a transaction has been aborted.
}

void QSPI::waitBusy()
{
    // wait till QUADSPI has completed the current communication with the flash.
    while (QUADSPI->SR & (1 << QUADSPI_SR_BUSY_Pos))
    {
        ;
    }
}

void QSPI::waitTransfer()
{
    // by setting data lenght register (DLR) you set how many bytes are expected
    // from memory. wait till all expected bytes have been tranferred.
    while (!(QUADSPI->SR & (1 << QUADSPI_SR_TCF_Pos)))
    {
        ;
    }

    // reset transfer complete flag (TCF)
    QUADSPI->FCR &= ~(1 << QUADSPI_FCR_CTCF_Pos);
}

qspi_flash::qspi_flash() { ; }

bool qspi_flash::test()
{
    // check if memory has been initialised
    if (initialised == false)
    {
        return false;
    }

    return readID() == DEVICE_ID ? true : false;
}

uint8_t qspi_flash::read_status_reg()
{

    // status register can be read at any time and during every kind of
    // operation.

    // check if memory has been initialised
    if (initialised == false)
    {
        return 0;
    }

    // reset peripheral
    QSPI::abort_reset();

    QUADSPI->CCR |= 1 << QUADSPI_CCR_FMODE_Pos |   // Indirect read mode
                    1 << QUADSPI_CCR_DMODE_Pos |   // Data on 1-wire
                    0 << QUADSPI_CCR_ABMODE_Pos |  // No alternate bytes
                    0 << QUADSPI_CCR_ADMODE_Pos |  // No address
                    1 << QUADSPI_CCR_IMODE_Pos;    // Instruction on 1-wire

    // Expect to receive 1 byte (flash status register)
    QUADSPI->DLR = 0;

    // enable peripheral
    QSPI::enable();

    // Trigger communication start by writing the instruction
    QUADSPI->CCR |= Commands::READ_STATUS_REG << QUADSPI_CCR_INSTRUCTION_Pos;

    // wait till data trasfer is complete
    QSPI::waitTransfer();

    // read status reg value from data register
    uint32_t value = (uint8_t)QUADSPI->DR;

    // disable peripheral
    QSPI::disable();

    return value;
}

void qspi_flash::write_enable()
{

    // 1 send wren command
    // 2 read status register
    // 3 wait for bit WEL = 1

    // check if memory has been initialised
    if (initialised == false)
    {
        return;
    }

    // reset peripheral
    QSPI::abort_reset();

    // indirect write mode, istruction on 1 wire, no data, no address, no
    // alternate bytes
    QUADSPI->CCR |= 1 << QUADSPI_CCR_IMODE_Pos;

    // enable peripheral
    QSPI::enable();

    // start communication writing the instruction to CCR register - write
    // enable command
    QUADSPI->CCR |= Commands::WRITE_ENABLE;

    // wait for the communication to end
    QSPI::waitBusy();

    // disable peripheral
    QSPI::disable();

    // check status register until bit WEL = 1
    uint8_t status = read_status_reg();
    while (!(status & (1 << 1)))
    {
        Thread::sleep(1);
        status = read_status_reg();
    }
}

void qspi_flash::init()
{
    // init QUADSPI peripheral in order to communicate with flash memory
    QSPI::init();

    // set flag intialised device
    initialised = true;
}

uint32_t qspi_flash::readID()
{

    // check if memory has been initialised
    if (initialised == false)
    {
        return 0;
    }

    // reset peripheral
    QSPI::abort_reset();

    QUADSPI->CCR |= 1 << QUADSPI_CCR_FMODE_Pos |   // Indirect read mode
                    1 << QUADSPI_CCR_DMODE_Pos |   // Data on 1-wire
                    0 << QUADSPI_CCR_ABMODE_Pos |  // No alternate bytes
                    0 << QUADSPI_CCR_ADMODE_Pos |  // No address
                    1 << QUADSPI_CCR_IMODE_Pos;    // Instruction on 1-wire

    // Expect to receive 3 bytes regarding ID of the flash
    QUADSPI->DLR = 2;

    // enable peripheral
    QSPI::enable();

    // Trigger communication by writing read ID command into CCR register
    QUADSPI->CCR |= Commands::READ_ID << QUADSPI_CCR_INSTRUCTION_Pos;

    // wait till communication is ended
    QSPI::waitTransfer();

    // sus: even if some bytes have been received, the FIFO need some time to
    // store them.
    Thread::sleep(1);

    // if there are some bytes in quadspi buffer (FIFO), read them
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

    // 1 send wrid command
    // 2 read status register
    // 3 wait for bit WEL = 0

    // check if memory has been initialised
    if (initialised == false)
    {
        return;
    }

    // reset peripheral
    QSPI::abort_reset();

    // indirect write mode, istruction on 1 wire, no data, no address, no
    // alternate bytes
    QUADSPI->CCR |= 1 << QUADSPI_CCR_IMODE_Pos;

    // enable peripheral
    QSPI::enable();

    // start communication writing write_disable command to CCR register
    QUADSPI->CCR |= Commands::WRITE_DISABLE;

    // wait till the communication has ended
    QSPI::waitBusy();

    // disable peripheral
    QSPI::disable();

    // check status register till bit WEL = 0
    uint8_t status = read_status_reg();
    while (status & (1 << 1))
    {
        Thread::sleep(1);
        status = read_status_reg();
    }
}

bool qspi_flash::isInProgress()
{

    // check if memory is currently executing some operation.
    // bit WIP in flash status register is set if a program/erase/write to
    // registers is being executed.
    // 1 read flash status register
    // 2 bit WIP = 1: operation in progress  |  WIP = 0 no one operation in
    // progress.

    // check if memory has been initialised
    if (initialised == false)
    {
        return false;
    }

    uint8_t status_reg = read_status_reg();
    return (status_reg & 1) ? true : false;
}

void qspi_flash::waitProgress()
{

    // check if memory has been initialised
    if (initialised == false)
    {
        return;
    }

    // wait till any operation is in progress
    while (isInProgress())
    {
        Thread::sleep(1);
    }
}

uint8_t qspi_flash::read_byte(uint32_t address)
{

    // 1 send READ command
    // 2 send 3-byte address
    // 3 read only the first byte of data
    // in this case, since no data is needed, the communication starts whenever
    // the adress register (QUADSPI->DR) is updated.

    // check if memory has been initialised
    if (initialised == false)
    {
        return 0;
    }

    // check on correct address range
    if (address > FlashMemory::MEMORY_SIZE)
        return 0;

    // reset peripheral
    QSPI::abort_reset();

    QUADSPI->CCR |= 1 << QUADSPI_CCR_FMODE_Pos |  // Indirect read mode
                    QUADSPI_CCR_DMODE_0 |         // data on a single  line
                    QUADSPI_CCR_ADSIZE_1 |        // 3-byte (24 bit) address
                    QUADSPI_CCR_ADMODE_0 |        // address on a single line
                    QUADSPI_CCR_IMODE_0;  // instruction on a single line

    // send read command
    QUADSPI->CCR |= Commands::READ;

    // just 1 byte of data is supposed to be transferred
    QUADSPI->DLR = 0;

    // enable peripheral
    QSPI::enable();

    // start communication by specifing the address
    QUADSPI->AR = address;

    // wait the expected byte has been transferred
    QSPI::waitTransfer();

    // read byte value from data register
    uint8_t value = (uint8_t)QUADSPI->DR;

    // disable peripheral
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

    // check if memory has been initialised
    if (initialised == false)
    {
        return false;
    }

    // wait for any ongoing operation
    waitProgress();

    // enable data writing
    write_enable();

    // reset peripheral
    QSPI::abort_reset();

    // indirect write mode, no address, no data. all on one wire.
    QUADSPI->CCR |= QUADSPI_CCR_IMODE_0;  // istruction on one wire

    // enable peripheral
    QSPI::enable();

    // write ERASE_CHIP command into CCR and start the communication
    QUADSPI->CCR |= Commands::ERASE_CHIP;

    // wait till the communication has ended
    QSPI::waitBusy();

    // disable peripheral
    QSPI::disable();

    // wait till current erase operation has ended
    waitProgress();

    // disable data writing
    write_disable();

    // return the result of chip erase operation
    return check_erase();
}

bool qspi_flash::sector_erase(uint32_t address)
{

    // 1 wait until the memory has finished any operation in progress
    // 2 write_enable command
    // 3 erase sector command
    // 4 wait till flash has completed the operation

    // check if memory has been initialised
    if (initialised == false)
    {
        return false;
    }

    // check on address range
    if ((address < 0) || (address > FlashMemory::MEMORY_SIZE))
        return false;

    // wait for any ongoing operation
    waitProgress();

    // enable data writing
    write_enable();

    // reset peripheral
    QSPI::abort_reset();

    // indirect write mode, 3-byte address , no data. all on one wire.
    QUADSPI->CCR |= QUADSPI_CCR_IMODE_0 |   // istruction on one wire
                    QUADSPI_CCR_ADSIZE_1 |  // 3-byte address
                    QUADSPI_CCR_ADMODE_0;   // address on a single line

    // enable peripheral
    QSPI::enable();

    // add sector erase command to CCR register
    QUADSPI->CCR |= Commands::SECTOR_ERASE;

    // start communication by writing the address in QUADSPI->AR
    QUADSPI->AR = address;

    // wait for the transaction to end
    QSPI::waitBusy();

    // disable data writing
    QSPI::disable();

    // wait till current erase operation has ended
    waitProgress();

    // disable data writing
    write_disable();

    // check on result of the last operation
    return check_erase();
}

bool qspi_flash::block32_erase(uint32_t address)
{

    // erase a 32K block of data, any address of the block is valid
    // 1 wait until the memory has finished any operation in progress
    // 2 write_enable command
    // 3 erase block_32 command
    // 4 wait till flash has completed the operation

    // check if memory has been initialised
    if (initialised == false)
    {
        return false;
    }

    // check on correct address range
    if ((address < 0) || (address > FlashMemory::MEMORY_SIZE))
        return false;

    // wait till any ongiong operation has ended
    waitProgress();

    // enable data writing
    write_enable();

    // reset peripheral
    QSPI::abort_reset();

    // indirect write mode, 3-byte address, no data. all on one wire.
    QUADSPI->CCR |= QUADSPI_CCR_IMODE_0 |   // istruction on one wire
                    QUADSPI_CCR_ADSIZE_1 |  // 3-byte address
                    QUADSPI_CCR_ADMODE_0;   // address on a single line

    // enable peripheral
    QSPI::enable();

    // add block_32_erase command to CCR
    QUADSPI->CCR |= Commands::BLOCK_32_ERASE;

    // start communication by writing the address in QUADSPI->AR
    QUADSPI->AR = address;

    // wait till communication has ended
    QSPI::waitBusy();

    // disable peripheral
    QSPI::disable();

    // wait till the current erase operation has been executed
    waitProgress();

    // disable data writing
    write_disable();

    // check on the result
    return check_erase();
}

bool qspi_flash::block64_erase(uint32_t address)
{

    // erase a 64K block of data, any address of the block is valid
    // 1 wait until the memory has finished any operation in progress
    // 2 write_enable command
    // 3 erase block_64 command
    // 4 wait till flash has completed the operation

    // check if memory has been initialised
    if (initialised == false)
    {
        return false;
    }

    // check on correct address range
    if ((address < 0) || (address > FlashMemory::MEMORY_SIZE))
        return false;

    // wait for any ongoing operation to end
    waitProgress();

    // enable data writing
    write_enable();

    // reset peripheral
    QSPI::abort_reset();

    // indirect write mode, 3-byte address, no data. all on one wire.
    QUADSPI->CCR |= QUADSPI_CCR_IMODE_0 |   // istruction on one wire
                    QUADSPI_CCR_ADSIZE_1 |  // 3-byte address
                    QUADSPI_CCR_ADMODE_0;   // address on a single line

    // enable peripheral
    QSPI::enable();

    // add BLOCK_64_ERASE command to CCR
    QUADSPI->CCR |= Commands::BLOCK_64_ERASE;

    // start communication by writing the address in QUADSPI->AR
    QUADSPI->AR = address;

    // wait till communication is ended
    QSPI::waitBusy();

    // didable peripheral
    QSPI::disable();

    // wait till the current erase operation has been completed
    waitProgress();

    // disable data writing
    write_disable();

    // check on result
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

    // check if memory has been initialised
    if (initialised == false)
    {
        return false;
    }

    // check on address range
    if ((address < 0) || (address > FlashMemory::MEMORY_SIZE))
        return false;

    // wait for any operation in progress
    waitProgress();

    // enable data writing
    write_enable();

    // reset peripheral
    QSPI::abort_reset();

    // idirect write mode
    QUADSPI->CCR |= QUADSPI_CCR_DMODE_0 |   // data on a single line
                    QUADSPI_CCR_ADSIZE_1 |  // address size 24 bit
                    QUADSPI_CCR_ADMODE_0 |  // address on a single line
                    QUADSPI_CCR_IMODE_0;    // instruction on a single line

    // enable peripheral
    QSPI::enable();

    // add PAGE_PROGRAM command to CCR
    QUADSPI->CCR |= Commands::PAGE_PROGRAM;

    // add address
    QUADSPI->AR = address;

    // trigger the communication by writing into data register
    QUADSPI->DR = data;

    // wait for the communication to end
    QSPI::waitBusy();

    // wait till the current program operation has ended
    waitProgress();

    // disable data writing
    write_disable();

    // if verify = true, double check on data saved
    if (verify == true)
    {
        if (read_byte(address) != data)
            return false;
    }

    // check on result
    return check_program();
}

uint8_t qspi_flash::read_security_reg()
{

    // security register can be read at any time and during every kind of
    // operation.
    // 1 - send read security register command
    // 2 - receive one byte of data containing the value of the register

    // check if memory has been initialised
    if (initialised == false)
    {
        return 0;
    }

    // reset peripheral
    QSPI::abort_reset();

    QUADSPI->CCR |= QUADSPI_CCR_FMODE_0 |  // Indirect read mode
                    QUADSPI_CCR_DMODE_0 |  // data on one wire
                    QUADSPI_CCR_IMODE_0;   // instruction on one wire

    // Expect to receive 1 byte (flash security register value)
    QUADSPI->DLR = 0;

    // enable peripheral
    QSPI::enable();

    // start communication by adding READ_SECURITY_REG command to CCR
    QUADSPI->CCR |= Commands::READ_SECURITY_REG;

    // wait till data trasfer is complete
    QSPI::waitTransfer();

    // sus! same sussata of readID() function
    Thread::sleep(1);

    // read register value from data register
    uint32_t value = (uint8_t)QUADSPI->DR;

    // disable peripheral
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

    // commands sequence: RSTEN >> wait 1ms >> RST >> wait 1ms

    // check if memory has been initialised
    if (initialised == false)
    {
        return;
    }

    // -------------------- send RSTEN command -------------------------
    // wait till any ongoing operation with the flash is ended
    waitProgress();

    // reset peripheral
    QSPI::abort_reset();

    // indirect write mode, no data, no address, instruction on a single line
    QUADSPI->CCR |= QUADSPI_CCR_IMODE_0;

    // enable peripheral
    QSPI::enable();

    // start the communication by adding RESET_ENABLE command to CCR
    QUADSPI->CCR |= Commands::RESET_ENABLE;

    // wait for the communication to end
    QSPI::waitBusy();

    // disable peripheral
    QSPI::disable();

    // let's give the flash some time to recognise reset_enable command
    Thread::sleep(1);

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
    // security register of memory:
    // true  - erase operation has succeded
    // false - erase operation has failed

    // check if memory has been initialised
    if (initialised == false)
    {
        return false;
    }

    uint8_t reg = read_security_reg();
    return reg & (1 << 6) ? false : true;
}

bool qspi_flash::check_program()
{

    // ATTTENTION! - this function check only if the last operation has been
    // completed, so it's not so reliable ! check for bit P_FAIL in security
    // register of memory:
    // true  - erase operation has succeded
    // false - erase operation has failed

    // check if memory has been initialised
    if (initialised == false)
    {
        return false;
    }

    uint8_t reg = read_security_reg();
    return reg & (1 << 5) ? false : true;
}

bool qspi_flash::read_sector(std::vector<uint8_t>& vector, uint32_t sector_num)
{

    // read an entire sector of the flash and then copy it into a std::vector
    // that will modify the vector and his elements.
    // that fix size and capacity of the vector to SECTOR_SIZE.

    // check if memory has been initialised
    if (initialised == false)
    {
        return false;
    }

    // check on correct sector_num range
    if (sector_num < 0 || sector_num > SECTORS_NUM)
    {
        return false;
    }

    // reserve enough bytes to retain a sector of the flash, also
    // make sure that size parameter of vector match with the capacity.
    if (vector.capacity() < SECTOR_SIZE)
    {
        vector.reserve(SECTOR_SIZE);
        vector.resize(vector.capacity());
    }

    // if vector capacity is larger than the size of a sector
    if (vector.capacity() >= SECTOR_SIZE)
    {
        vector.resize(SECTOR_SIZE);
        vector.shrink_to_fit();
    }

    // computing correct address of the sector
    uint32_t addr = SECTOR_SIZE * sector_num;

    // read the sector and copy its data to vector
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

    // program a vector (max size = 256 bytes) starting by a specific page
    // address. WEL bit is set to zero automatically after the program
    // operation. the address specified must be a starting
    // address of a page !!!!

    // check if memory has been initialised
    if (initialised == false)
    {
        return false;
    }

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

    // enable data writing
    write_enable();

    // reset peripheral
    QSPI::abort_reset();

    // idirect write mode
    QUADSPI->CCR |= QUADSPI_CCR_DMODE_0 |   // data on a single line
                    QUADSPI_CCR_ADSIZE_1 |  // address size 24 bit
                    QUADSPI_CCR_ADMODE_0 |  // address on a single line
                    QUADSPI_CCR_IMODE_0;    // instruction on a single line

    // enable peripheral
    QSPI::enable();

    // add page program command to CCR
    QUADSPI->CCR |= Commands::PAGE_PROGRAM;

    // set number of bytes to be transferred - 1
    QUADSPI->DLR = vector.size() - 1;

    // adding starting address
    QUADSPI->AR = start_address;

    // load data vector into the QUADSPI FIFO (buffer)
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

    // wait for the end of communication
    QSPI::waitBusy();

    // wait till the end of current program operation
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

    // check on last program operation result
    return check_program();
}

bool qspi_flash::write_vector(std::vector<uint8_t>& vector, uint32_t sector_num,
                              bool verify_write)
{

    // store a vector of bytes into the flash memory

    // check if memory has been initialised
    if (initialised == false)
    {
        return false;
    }

    // wrong sector_num specified
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

    // compute starting address
    uint32_t const start_address = SECTOR_SIZE * sector_num;

    // compute number of sectors needed to store the vector, then add one more
    // to store the last part of vector
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

    // compute the number of pages needed to store the entire vector, then add
    // one more to store the vector last part.
    uint32_t pages_needed = vector.size() / PAGE_SIZE;
    if ((vector.size() % PAGE_SIZE) != 0)
        pages_needed += 1;

    // create a copy vector with capacity as a page size
    std::vector<uint8_t> v;
    v.reserve(PAGE_SIZE);
    v.resize(0);

    // for every page needed, first copy data bytes from "vector" to a copy
    // vector "v" and then program the page.
    uint32_t page = 0;
    uint32_t elem = 0;
    for (page = 0; page < pages_needed; page++)
    {
        v.resize(0);
        uint32_t start_elem = page * PAGE_SIZE;
        // copying 256 bytes from "vector" to "v"
        for (elem = start_elem;
             (elem < start_elem + PAGE_SIZE) && (elem < vector.size()); elem++)
        {
            v.push_back(vector[elem]);
        }

        // program the page stored into "v" on flash
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