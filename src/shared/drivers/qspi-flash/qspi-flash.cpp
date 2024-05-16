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

void qspi_flash::enable() { Qspi->CR |= QUADSPI_CR_EN; }

void qspi_flash::disable() { Qspi->CR &= ~QUADSPI_CR_EN; }

void qspi_flash::abort_reset()
{

    // abort possible ongoing command
    Qspi->CR |= QUADSPI_CR_ABORT;

    // Wait while aborted
    uint32_t dt = 0;  // timeout
    while (Qspi->CR & QUADSPI_CR_ABORT)
    {
        dt = dt + 1;
        if (dt > 10000)
        {
            return;
        }
    }

    // to be sure that the peripheral is disabled
    disable();

    // reset configuration register
    Qspi->CCR = 0;

    // transfer flag (TCF) will reset automatically and QUADSPI FIFO is flushed
    // if a transaction has been aborted.
}

void qspi_flash::waitBusy()
{
    // wait till QUADSPI has completed the current communication with the flash
    uint32_t dt = 0;  // timeout
    while (Qspi->SR & (1 << QUADSPI_SR_BUSY_Pos))
    {
        dt = dt + 1;
        if (dt > 20000)
        {
            return;
        }
    }
}

void qspi_flash::waitTransfer()
{
    // by setting data lenght register (DLR) you set how many bytes are expected
    // from memory. wait till all expected bytes have been tranferred.
    uint32_t dt = 0;  // timeout
    while (!(Qspi->SR & (1 << QUADSPI_SR_TCF_Pos)))
    {
        dt = dt + 1;
        if (dt > 20000)
        {
            return;
        }
    }

    // reset transfer complete flag (TCF)
    Qspi->FCR &= ~(1 << QUADSPI_FCR_CTCF_Pos);
}

qspi_flash::qspi_flash(QUADSPI_TypeDef* qspi)
{

    Qspi = qspi;

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
}

bool qspi_flash::test()
{
    // check if memory has been initialised
    if (initialised == false)
    {
        return false;
    }

    return readID() == DEVICE_ID;
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
    abort_reset();

    Qspi->CCR |= 1 << QUADSPI_CCR_FMODE_Pos |   // Indirect read mode
                 1 << QUADSPI_CCR_DMODE_Pos |   // Data on 1-wire
                 0 << QUADSPI_CCR_ABMODE_Pos |  // No alternate bytes
                 0 << QUADSPI_CCR_ADMODE_Pos |  // No address
                 1 << QUADSPI_CCR_IMODE_Pos;    // Instruction on 1-wire

    // Expect to receive 1 byte (flash status register)
    Qspi->DLR = 0;

    // enable peripheral
    enable();

    // Trigger communication start by writing the instruction
    Qspi->CCR |= Commands::READ_STATUS_REG << QUADSPI_CCR_INSTRUCTION_Pos;

    // wait till data trasfer is complete
    waitTransfer();

    // read status reg value from data register
    uint32_t value = (uint8_t)Qspi->DR;

    // disable peripheral
    disable();

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
    abort_reset();

    // indirect write mode, istruction on 1 wire, no data, no address, no
    // alternate bytes
    Qspi->CCR |= 1 << QUADSPI_CCR_IMODE_Pos;

    // enable peripheral
    enable();

    // start communication writing the instruction to CCR register - write
    // enable command
    Qspi->CCR |= Commands::WRITE_ENABLE;

    // wait for the communication to end
    waitBusy();

    // disable peripheral
    disable();

    // check status register until bit WEL = 1
    uint8_t status = read_status_reg();
    uint8_t dt     = 0;  // timeout
    while (!(status & (1 << 1)))
    {
        Thread::sleep(1);
        dt = dt + 1;
        if (dt >= 5)
        {
            return;
        }
        status = read_status_reg();
    }
}

void qspi_flash::init()
{

    // init peripheral clock
    ClockUtils::enablePeripheralClock((QUADSPI_TypeDef*)QSPI_BASE);

    RCC_SYNC();

    Thread::sleep(1);

    // abort any operation
    abort_reset();

    // reset configuration registers
    Qspi->CR  = 0;
    Qspi->DCR = 0;
    Qspi->CCR = 0;
    Qspi->DLR = 0;

    // reset transfer complete flag (TCF)
    Qspi->FCR &= ~(1 << QUADSPI_FCR_CTCF_Pos);

    // peripheral default initialization
    Qspi->CR |=
        QUADSPI_CR_SSHIFT |             // Wait a full cycle to read
        3 << QUADSPI_CR_PRESCALER_Pos;  // QSPI clock = 216MHz / 4 = 54MHz

    // set the memory chip size - it must be always setted before any read
    // operation.
    Qspi->DCR |=
        21 << QUADSPI_DCR_FSIZE_Pos;  // Flash size 32Mb = 4MB = 2^(21+1) bytes

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
    abort_reset();

    Qspi->CCR |= 1 << QUADSPI_CCR_FMODE_Pos |   // Indirect read mode
                 1 << QUADSPI_CCR_DMODE_Pos |   // Data on 1-wire
                 0 << QUADSPI_CCR_ABMODE_Pos |  // No alternate bytes
                 0 << QUADSPI_CCR_ADMODE_Pos |  // No address
                 1 << QUADSPI_CCR_IMODE_Pos;    // Instruction on 1-wire

    // Expect to receive 3 bytes regarding ID of the flash
    Qspi->DLR = 2;

    // enable peripheral
    enable();

    // Trigger communication by writing read ID command into CCR register
    Qspi->CCR |= Commands::READ_ID << QUADSPI_CCR_INSTRUCTION_Pos;

    // wait till communication is ended
    waitTransfer();

    // sus: even if some bytes have been received, the FIFO need some time to
    // store them.
    Thread::sleep(1);

    // if there are some bytes in quadspi buffer (FIFO), read them
    if ((Qspi->SR & QUADSPI_SR_FLEVEL) >> 8)
    {
        uint32_t myID = Qspi->DR;

        disable();

        // the ID bytes order must be inverted
        uint8_t lb = myID;                   // lowest byte
        uint8_t mb = (myID >> 8) & (255U);   // middle byte
        uint8_t hb = (myID >> 16) & (255U);  // highest byte

        myID = (lb << 16) | (mb << 8) | (hb);

        return myID;
    }
    else
    {
        disable();
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
    abort_reset();

    // indirect write mode, istruction on 1 wire, no data, no address, no
    // alternate bytes
    Qspi->CCR |= 1 << QUADSPI_CCR_IMODE_Pos;

    // enable peripheral
    enable();

    // start communication writing write_disable command to CCR register
    Qspi->CCR |= Commands::WRITE_DISABLE;

    // wait till the communication has ended
    waitBusy();

    // disable peripheral
    disable();

    // check status register till bit WEL = 0
    uint8_t status = read_status_reg();
    uint8_t dt     = 0;  // timeout
    while (status & (1 << 1))
    {
        Thread::sleep(1);
        dt = dt + 1;
        if (dt >= 5)
        {
            return;
        }
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

uint8_t qspi_flash::read_byte(uint32_t address)
{

    // 1 send READ command
    // 2 send 3-byte address
    // 3 read only the first byte of data
    // in this case, since no data is needed, the communication starts whenever
    // the adress register (Qspi->DR) is updated.

    // check if memory has been initialised
    if (initialised == false)
    {
        return 0;
    }

    // check on correct address range
    if (address > FlashMemory::MEMORY_SIZE)
        return 0;

    // reset peripheral
    abort_reset();

    Qspi->CCR |= 1 << QUADSPI_CCR_FMODE_Pos |  // Indirect read mode
                 QUADSPI_CCR_DMODE_0 |         // data on a single  line
                 QUADSPI_CCR_ADSIZE_1 |        // 3-byte (24 bit) address
                 QUADSPI_CCR_ADMODE_0 |        // address on a single line
                 QUADSPI_CCR_IMODE_0;          // instruction on a single line

    // send read command
    Qspi->CCR |= Commands::READ;

    // just 1 byte of data is supposed to be transferred
    Qspi->DLR = 0;

    // enable peripheral
    enable();

    // start communication by specifing the address
    Qspi->AR = address;

    // wait the expected byte has been transferred
    waitTransfer();

    // read byte value from data register
    uint8_t value = (uint8_t)Qspi->DR;

    // disable peripheral
    disable();

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

    // if chip_erase operation will not be completed properly resulting in a
    // timeout event and a forced reset (after some seconds by the command),
    // some post-operations may fail!

    // check if memory has been initialised
    if (initialised == false)
    {
        return false;
    }

    // enable data writing
    write_enable();

    // reset peripheral
    abort_reset();

    // indirect write mode, no address, no data. all on one wire.
    Qspi->CCR |= QUADSPI_CCR_IMODE_0;  // istruction on one wire

    // enable peripheral
    enable();

    // write ERASE_CHIP command into CCR and start the communication
    Qspi->CCR |= Commands::ERASE_CHIP;

    // wait till the communication has ended
    waitBusy();

    // disable peripheral
    disable();

    // wait till current erase operation has ended
    uint32_t dt = 0;  // timeout
    while (isInProgress())
    {
        Thread::sleep(1);
        dt = dt + 1;
        if (dt >= 130000)  // (2 min and 10sec) max chip erase time = 2 min
        {
            software_reset();   // device forced reset to default status
            Thread::sleep(20);  // recovery time = 12 ms
            return false;
        }
    }

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

    // enable data writing
    write_enable();

    // reset peripheral
    abort_reset();

    // indirect write mode, 3-byte address , no data. all on one wire.
    Qspi->CCR |= QUADSPI_CCR_IMODE_0 |   // istruction on one wire
                 QUADSPI_CCR_ADSIZE_1 |  // 3-byte address
                 QUADSPI_CCR_ADMODE_0;   // address on a single line

    // enable peripheral
    enable();

    // add sector erase command to CCR register
    Qspi->CCR |= Commands::SECTOR_ERASE;

    // start communication by writing the address in Qspi->AR
    Qspi->AR = address;

    // wait for the transaction to end
    waitBusy();

    // disable data writing
    disable();

    // wait till current erase operation has ended
    uint32_t dt = 0;  // timeout
    while (isInProgress())
    {
        Thread::sleep(1);
        dt = dt + 1;
        if (dt >= 1000)  // max sector erase cycle time = 240 ms
        {
            software_reset();  // device forced reset to default status
            return false;
        }
    }

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

    // enable data writing
    write_enable();

    // reset peripheral
    abort_reset();

    // indirect write mode, 3-byte address, no data. all on one wire.
    Qspi->CCR |= QUADSPI_CCR_IMODE_0 |   // istruction on one wire
                 QUADSPI_CCR_ADSIZE_1 |  // 3-byte address
                 QUADSPI_CCR_ADMODE_0;   // address on a single line

    // enable peripheral
    enable();

    // add block_32_erase command to CCR
    Qspi->CCR |= Commands::BLOCK_32_ERASE;

    // start communication by writing the address in Qspi->AR
    Qspi->AR = address;

    // wait till communication has ended
    waitBusy();

    // disable peripheral
    disable();

    // wait till current erase operation has ended
    uint32_t dt = 0;  // timeout
    while (isInProgress())
    {
        Thread::sleep(1);
        dt = dt + 1;
        if (dt >= 3000)  // block_32 erase cycle max time = 1.8s
        {
            software_reset();  // device forced reset to default status
            return false;
        }
    }

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

    // enable data writing
    write_enable();

    // reset peripheral
    abort_reset();

    // indirect write mode, 3-byte address, no data. all on one wire.
    Qspi->CCR |= QUADSPI_CCR_IMODE_0 |   // istruction on one wire
                 QUADSPI_CCR_ADSIZE_1 |  // 3-byte address
                 QUADSPI_CCR_ADMODE_0;   // address on a single line

    // enable peripheral
    enable();

    // add BLOCK_64_ERASE command to CCR
    Qspi->CCR |= Commands::BLOCK_64_ERASE;

    // start communication by writing the address in Qspi->AR
    Qspi->AR = address;

    // wait till communication is ended
    waitBusy();

    // didable peripheral
    disable();

    // wait till current erase operation has ended
    uint32_t dt = 0;  // timeout
    while (isInProgress())
    {
        Thread::sleep(1);
        dt = dt + 1;
        if (dt >= 4500)  // erase block_64K cycle max time = 3.5s
        {
            software_reset();  // device forced reset to default status
            return false;
        }
    }

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
    {
        return false;
    }

    // enable data writing
    write_enable();

    // reset peripheral
    abort_reset();

    // idirect write mode
    Qspi->CCR |= QUADSPI_CCR_DMODE_0 |   // data on a single line
                 QUADSPI_CCR_ADSIZE_1 |  // address size 24 bit
                 QUADSPI_CCR_ADMODE_0 |  // address on a single line
                 QUADSPI_CCR_IMODE_0;    // instruction on a single line

    // enable peripheral
    enable();

    // add PAGE_PROGRAM command to CCR
    Qspi->CCR |= Commands::PAGE_PROGRAM;

    // add address
    Qspi->AR = address;

    // trigger the communication by writing into data register
    Qspi->DR = data;

    // wait for the communication to end
    waitBusy();

    // wait till current program operation has ended
    uint32_t dt = 0;  // timeout
    while (isInProgress())
    {
        dt = dt + 1;
        if (dt >= 5000)  // max program byte cycle time = 100us
        {
            software_reset();  // device forced reset to default status
            return false;
        }
    }

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
    abort_reset();

    Qspi->CCR |= QUADSPI_CCR_FMODE_0 |  // Indirect read mode
                 QUADSPI_CCR_DMODE_0 |  // data on one wire
                 QUADSPI_CCR_IMODE_0;   // instruction on one wire

    // Expect to receive 1 byte (flash security register value)
    Qspi->DLR = 0;

    // enable peripheral
    enable();

    // start communication by adding READ_SECURITY_REG command to CCR
    Qspi->CCR |= Commands::READ_SECURITY_REG;

    // wait till data trasfer is complete
    waitTransfer();

    // sus! same sussata of readID() function
    Thread::sleep(1);

    // read register value from data register
    uint32_t value = (uint8_t)Qspi->DR;

    // disable peripheral
    disable();

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

    // reset peripheral
    abort_reset();

    // indirect write mode, no data, no address, instruction on a single line
    Qspi->CCR |= QUADSPI_CCR_IMODE_0;

    // enable peripheral
    enable();

    // start the communication by adding RESET_ENABLE command to CCR
    Qspi->CCR |= Commands::RESET_ENABLE;

    // wait for the communication to end
    waitBusy();

    // disable peripheral
    disable();

    // let's give the flash some time to recognise reset_enable command
    Thread::sleep(1);

    // ------------------- send RST command --------------------------
    abort_reset();

    Qspi->CCR |= QUADSPI_CCR_IMODE_0;

    enable();

    Qspi->CCR |= Commands::RESET_MEMORY;

    waitBusy();

    disable();

    // wait for flash to go back in power-on default status
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

bool qspi_flash::read_sector(uint8_t* vector, const size_t size,
                             uint32_t sector_num)
{

    // read an entire sector of the flash and then copy it into a vector whose
    // size is equal or greater than the size of a sector.

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

    // vector's size must be large enough for a sector
    if (size < SECTOR_SIZE)
    {
        return false;
    }

    // computing correct address of the sector
    uint32_t addr = SECTOR_SIZE * sector_num;

    // read the sector and copy its data to vector
    uint32_t index = 0;
    for (index = 0; index < SECTOR_SIZE; index++)
    {
        vector[index] = read_byte(addr);
        addr++;
    }

    return true;
}

bool qspi_flash::page_program(const uint8_t* vector, const size_t size,
                              uint32_t start_address, bool verify)
{

    // program a vector (max size = 256 bytes) on flash memory starting by a
    // specific page address.
    // WEL bit is set to zero automatically after the program
    // operation. The address specified must be a starting
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

    // empty vector or null pointer
    if (size == 0 || vector == nullptr)
        return false;

    // vector bigger than a page size
    if (size > PAGE_SIZE)
        return false;

    // enable data writing
    write_enable();

    // reset peripheral
    abort_reset();

    // idirect write mode
    Qspi->CCR |= QUADSPI_CCR_DMODE_0 |   // data on a single line
                 QUADSPI_CCR_ADSIZE_1 |  // address size 24 bit
                 QUADSPI_CCR_ADMODE_0 |  // address on a single line
                 QUADSPI_CCR_IMODE_0;    // instruction on a single line

    // enable peripheral
    enable();

    // add page program command to CCR
    Qspi->CCR |= Commands::PAGE_PROGRAM;

    // set number of bytes to be transferred - 1
    Qspi->DLR = size - 1;

    // adding starting address
    Qspi->AR = start_address;

    // load data vector into the QUADSPI FIFO (buffer)
    uint16_t i = 0;
    for (i = 0; i < size; i++)
    {

        // if FIFO is full - wait till it has at least a byte available.
        uint32_t dt = 0;  // timeout
        while (((Qspi->SR & QUADSPI_SR_FLEVEL) >> 8) >= 32)
        {
            dt = dt + 1;
            if (dt >= 10000)
            {
                return false;
            }
        }

        // add a single byte to be sent into the QSPI FIFO
        ((uint8_t*)&Qspi->DR)[0] = static_cast<uint8_t>(vector[i]);
    }

    // wait for the end of communication
    waitBusy();

    // wait till current program operation has ended
    uint32_t dt = 0;  // timeout
    while (isInProgress())
    {
        Thread::sleep(1);
        dt = dt + 1;
        if (dt >= 50)  // max page program cycle time = 10ms
        {
            software_reset();  // device forced reset to default status
            return false;
        }
    }

    // if verify flag is set, double check on written data
    if (verify == true)
    {
        for (i = 0; i < size; i++)
        {
            if (read_byte(start_address + i) != vector[i])
                return false;
        }
    }

    // check on last program operation result
    return check_program();
}

bool qspi_flash::write(const uint8_t* vector, const size_t size,
                       uint32_t sector_num, bool verify_write)
{

    // check if memory has been initialised
    if (initialised == false)
    {
        return false;
    }

    // wrong sector_num specified
    if (sector_num >= SECTORS_NUM)
    {
        return false;
    }

    // check that vector is valid and not empty
    if ((vector == nullptr) || size == 0)
    {
        return false;
    }

    // if the vector is bigger than the flash capacity
    if (size > MEMORY_SIZE)
    {
        return false;
    }

    // if the whole vector is bigger than the rest of sectors starting by
    // sector_num
    if (size > (SECTOR_SIZE * (SECTORS_NUM - sector_num)))
    {
        return false;
    }

    // compute starting address
    uint32_t const start_address = SECTOR_SIZE * sector_num;

    // compute number of sectors needed to store the vector, then add one more
    // to store the last part of vector
    uint32_t sectors_needed = size / SECTOR_SIZE;
    if ((size % SECTOR_SIZE) != 0)
    {
        sectors_needed += 1;
    }

    // erase all sectors needed to store the entire vector
    uint32_t sector_index = 0;
    for (sector_index = 0; sector_index < sectors_needed; sector_index++)
    {
        if (sector_erase(SECTOR_SIZE * (sector_num + sector_index)) == false)
        {
            return false;
        }
    }

    // compute the number of pages needed to store the entire vector, then add
    // one more to store the vector last part
    uint32_t pages_needed = size / PAGE_SIZE;
    if ((size % PAGE_SIZE) != 0)
    {
        pages_needed += 1;
    }

    // split vector in pages and then program them on flash but the last one
    uint32_t page = 0;
    for (page = 0; page < pages_needed - 1; page++)
    {
        if (page_program(vector + (page * PAGE_SIZE), PAGE_SIZE,
                         start_address + (page * PAGE_SIZE), false) == false)
            return false;
    }

    // program the last page, cause probably it has a different size
    // than others.
    /*
     *  a little explanation of parameters:
     *  1 - make the pointer point to the start byte of last page.
     *  2 - is the size of last page which store the vector final part.
     *  3 - calculate the start address of last page.
     */
    if (page_program(
            vector + ((pages_needed - 1) * PAGE_SIZE), size % PAGE_SIZE,
            start_address + ((pages_needed - 1) * PAGE_SIZE), false) == false)
    {
        return false;
    }

    // if verify_write is true:
    // check that the bytes written into the flash match with the ones in the
    // vector.
    if (verify_write)
    {
        uint32_t index = 0;
        uint32_t addr  = start_address;
        for (index = 0; index < size; index++)
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