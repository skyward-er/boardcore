/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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


#ifndef FLASHDRIVER_H
#define FLASHDRIVER_H

#include <miosix.h>
#include <Singleton.h>
#include <diagnostic/NewLogger.h>
#include <string>

using miosix::Thread;
using miosix::Mode;

using logging::logger;

namespace flashmemory {

static const uint32_t PAGES_PER_SUBSECTOR = 16U;
static const uint32_t PAGES_PER_SECTOR = 256U;
static const uint32_t SUBSECTORS_PER_SECTOR = 16U;

static const uint32_t SUBSECTORS_NUM = 16384U;
static const uint32_t SECTORS_NUM = 1024U;
static const uint32_t PAGE_SIZE = 256U;

static const uint32_t SUBSECTOR_SIZE = PAGE_SIZE*PAGES_PER_SUBSECTOR;
static const uint32_t SECTOR_SIZE = PAGE_SIZE*PAGES_PER_SECTOR;

static const uint32_t BANK_SIZE = SECTOR_SIZE*SECTORS_NUM/2;
static const uint32_t MEMORY_SIZE = SECTOR_SIZE*SECTORS_NUM;


/**
 * 	@brief Definitions for results of various flash operations.
 *
 * 	Definitions for result flags of various flash operations.
 */
enum OpResultFlags
{
	RESULT_OK 				= 0x00,


	RESULT_F_OUT_OF_MEMORY 	= 0x01,
	RESULT_F_CHECK_FAIL 		= 0x04,


	//Bits reserved for future use
	RESULT_F_UNUSED_1			= 0x08,
	RESULT_F_UNUSED_2 		= 0x40,
	RESULT_F_UNUSED_3 		= 0x80,


	/**
	 * These bits are in the same positions of the corresponding bits in the
	 * FLAG STATUS REGISTER
	 */
	RESULT_F_PROTECTION_ERROR = 0x02,
	RESULT_F_PROGRAM_ERROR 	= 0x10,
	RESULT_F_ERASE_ERROR		= 0x20
};


/*
 * TODO:
 * -Controlla significato bit Vpp nel Flag status register
 * -Casi in cui eseguire _write_disable()
 * -Velocizzare il clock software per il factory reset
 * -Controlla se bisogna davvero leggere due volte il flag_status_register per
 * 	vedere se una scrittura ad un registro e' stata completata
 *
 */

template<typename Bus>
class FlashDriver : Singleton<FlashDriver<Bus>>
{
  typedef Singleton<FlashDriver<Bus>> SingletonType;

  friend class Singleton<FlashDriver<Bus>>;

	template<typename>
	friend class FlashDriverTest;

public:

	/**
	 * @brief Read n bytes into a buffer, starting from the specified address
	 *
	 * @param result
	 * @param address Starting address
	 * @param buf Buffer to read data into
	 * @param size Number of bytes to read
	 */
	void read(uint8_t *result, uint32_t address, uint8_t* buf, uint32_t size)
	{
		if(address + size > MEMORY_SIZE){
			*result = RESULT_F_OUT_OF_MEMORY;
			return;
		}

		uint32_t rsize;
		if(address < BANK_SIZE)
			rsize = std::min(size, BANK_SIZE - address);
		else
			rsize = size;

		uint8_t addr_buf[4];
		addrToBuf(addr_buf, address);

		Bus::read(READ, addr_buf, buf, 4, rsize);

		size -= rsize;

		//This means that we reached the end of the first die but we still need
		//to read some data.
		if(size > 0)
		{
			address += rsize;
			buf += rsize;

			addrToBuf(addr_buf, address);

			//Read the remaining bytes
			Bus::read(READ, addr_buf, buf, 4, size);
		}

		*result = RESULT_OK;
	}

	/**
	 * @brief Writes data on the flash starting from the specified address.
	 *
	 * @param address
	 * @param data
	 * @param size
	 */
	void write(uint8_t *result, uint32_t address, uint8_t* data, uint32_t size)
	{
		//Do not write outside of the memory
		if(address + size > MEMORY_SIZE)
		{
			*result = RESULT_F_OUT_OF_MEMORY;
			return;
		}

		uint8_t addr_buf[4], status;

		uint32_t next_page, wsize;

		do {
			next_page = (address / 256)*256 + 256;

			wsize = std::min(next_page - address, size);

			addrToBuf(addr_buf, address);
			writeEnable();
			Bus::write(PROGRAM_PAGE, addr_buf, data, 4, wsize);

			do {
				status = readFlagStatusReg();
			}while((status & FSR_PRG_ERS_CTRL) == 0);

			data += wsize;
			size -= wsize;
			address = next_page;

			*result = status & (FSR_PROGRAM | FSR_PROTECTION);
		}while(size > 0 && *result == RESULT_OK);

		//Clear flag status register if an error occurred
		if(*result != RESULT_OK)
		{
			clearFlagStatusReg();

			//Manually reset write_enable latch
			if((*result & RESULT_F_PROTECTION_ERROR) > 0)
			{
				writeDisable();
			}
		}
	}

	/**
	 * @brief Writes data on the flash starting from the specified address, then
	 * checks if everything was written correctly.
	 *
	 * @param address
	 * @param data
	 * @param size
	 */
	void writeAndCheck(uint8_t *result, uint32_t address, uint8_t* data,
			uint32_t size){ //TODO: size_t
		write(result,address,data,size);
		if(*result == RESULT_OK){
			uint8_t *check = new uint8_t[size];
			read(result, address, check, size);
			for(uint32_t i = 0; i < size; i++){
				if(*check++ != *data++){
					//Something was not written/read correctly.
					*result = RESULT_F_CHECK_FAIL;
					break;
				}
			}
		}
	}

	/**
	 * @brief Writes data on the flash starting from the specified address, only
	 * if the provided data fits in a single page.
	 *
	 * Writes data on the flash starting from the specified address, only
	 * if the provided data fits in a single page. If too much data is provided
	 * nothing will be written and result will be set to RESULT_F_OUT_OF_MEMORY
	 *
	 * @param result
	 * @param address
	 * @param data
	 * @param size
	 */
	void programPage(uint8_t *result, uint32_t address, uint8_t* data, uint32_t size)
	{
		//Do now wrap around a page and do not try to write outside of the memory
		uint32_t next_page = (address / 256)*256 + 256;
		if(address + size > next_page || address > MEMORY_SIZE)
		{
			*result = RESULT_F_OUT_OF_MEMORY;
			return;
		}

		uint8_t addr_buf[4], status;
		addrToBuf(addr_buf, address);

		writeEnable();
		Bus::write(PROGRAM_PAGE, addr_buf, data, 4, size);

		do {
			status = readFlagStatusReg();
		}while((status & FSR_PRG_ERS_CTRL) == 0);

		*result = status & (FSR_PROGRAM | FSR_PROTECTION);

		//Clear flag status register if an error has been encountered
		if(*result != RESULT_OK)
		{
			clearFlagStatusReg();

			//Manually reset write_enable latch
			if((*result & RESULT_F_PROTECTION_ERROR) > 0)
			{
				writeDisable();
			}
		}
	}

	/**
	 * @brief Enables or disables read only mode
	 */
	void setReadOnly(bool readonly = true)
	{
		read_only_ = readonly;
	}


	bool isReadOnly() const
	{
	  return read_only_;
	}

	/**
	 * @brief Enable the next erase operation. Call this before every erase op.
	 */
	void enableErase()
	{
		writeEnable();
	}

	/**
	 * @brief Erases the subsector containing the specified address. For the operation
	 * to be successful, enable_erase_op must be called before this.
	 *
	 * @warning Estimated execution time is slightly less than 1 second.
	 *
	 * @param result
	 * @param address
	 */
	void eraseSubsector(uint8_t* result, uint32_t address)
	{
		erase(result, SUBSECTOR_ERASE, address);
	}

	/**
	 * @brief Erases the sector containing the specified address. For the operation
	 * to be successful, enable_erase_op must be called before this.
	 * @warning Estimated execution time is about 2 seconds.
	 *
	 * @param result
	 * @param address
	 */
	void eraseSector(uint8_t* result, uint32_t address)
	{
		erase(result, SECTOR_ERASE, address);
	}

	/**
	 * @brief Erases the entire die containing the specified address. For the operation
	 * to be successful, enable_erase_op must be called before this.
	 * @warning Estimated execution time is more than 4 minutes.
	 *
	 * @param result
	 * @param die: 0 to erase the first die, 1 to erase the second.
	 */
	void eraseDie(uint8_t* result, uint8_t die)
	{
		if(die == 0){
			erase(result, DIE_ERASE, 0);
		}else if(die == 1){
			erase(result, DIE_ERASE, BANK_SIZE);
		}
	}

	/**
	 * Reads id information for this flash memory.
	 * @param buf Buffer with at least 20 bytes
	 */
	void readId(uint8_t* buf)
	{
		Bus::read(READ_ID, buf, 20);
	}


	/**
	 * @brief Software reset for the memory. All volatile bits are set to their
	 * default value.
	 */
	void softReset()
	{
		Bus::write(RESET_ENABLE);
		usleep(5);
		Bus::write(RESET_MEMORY);
		waitUntilReady();
	}

	/**
	 * If the flash memory is in a bad state and can't be recovered by any
	 * other means, run this, then set CONFIG_REGISTER to 0xFFFF.
	 */
	template <class CS, class MOSI, class CLK>
	static void factoryReset()
	{
		{
			miosix::FastInterruptDisableLock dLock;
			CS::mode(Mode::OUTPUT);
			MOSI::mode(Mode::OUTPUT);
			CLK::mode(Mode::OUTPUT);
		}
		MOSI::high();
		Thread::sleep(20);


		factoryResetSequence<CS, CLK>(7);
		factoryResetSequence<CS, CLK>(13);
		factoryResetSequence<CS, CLK>(17);
		factoryResetSequence<CS, CLK>(25);
		factoryResetSequence<CS, CLK>(33);

		MOSI::low();
		Thread::sleep(50);
		MOSI::high();
		factoryResetSequence<CS, CLK>(8);

		MOSI::low();
		CS::low();
	}

private:
	FlashDriver(){
	  waitUntilReady();

    uint8_t buf[20];
    readId(buf);

    if(buf[0] != 0x20)
    {
      logger.critical(logtag(), "Wrong id on FlashDriver instantiation");
    }

    writeStatusReg(0x00);

    clearFlagStatusReg();


    uint16_t config = readConfigReg();

    //Last bit to 0 to enable 4 byte address mode
    if(config != 0xFFFE) writeConfigReg(0xFFFE);
	}

	void erase(uint8_t *result, uint8_t erase_cmd, uint32_t address)
	{
		if(address > MEMORY_SIZE)
		{
			*result = RESULT_F_OUT_OF_MEMORY;
			return;
		}

		uint8_t addr_buf[4], status;
		addrToBuf(addr_buf, address);
		Bus::write(erase_cmd, addr_buf, 4);

		do {
			status = readFlagStatusReg();
		}while((status & FSR_PRG_ERS_CTRL) == 0);

		*result = status & (FSR_ERASE | FSR_PROTECTION);

		//Clear flag status register if an error has been encountered
		if(*result != RESULT_OK)
		{
			clearFlagStatusReg();

			//Manually reset write_enable latch
			if((*result & RESULT_F_PROTECTION_ERROR) > 0)
			{
				writeDisable();
			}
		}
	}

	/**
   * @brief Checks if a program operation is still in progress.
   * @param result
   */
  bool isBusy()
  {
    return readFlagStatusReg() & FSR_PRG_ERS_CTRL;
  }

  /**
   * @brief Waits until the memory is ready to perform a new write/erase op.
   * @param result
   */
  void waitUntilReady()
  {
    while((readFlagStatusReg() & FSR_PRG_ERS_CTRL) == 0)
    {

    }
  }

	uint8_t readStatusReg()
	{
		uint8_t ret = Bus::read(READ_STATUS_REG);
		return ret;
	}

	void writeStatusReg(uint8_t val)
	{
		writeEnable();
		Bus::write(WRITE_STATUS_REG, val);
		writeDisable();
		waitUntilReady();
	}

	uint8_t readFlagStatusReg()
	{
		uint8_t ret = Bus::read(READ_FLAG_STATUS_REG);
		return ret;
	}

	void clearFlagStatusReg()
	{
		Bus::write(CLEAR_FLAG_STATUS_REG);
	}

	uint16_t readConfigReg()
	{
		uint8_t buf[2];
		Bus::read(READ_NV_CONFIG_REG, buf, 2);

		uint16_t res = buf[0] | (buf[1] << 8);

		return res;
	}

	void writeConfigReg(uint16_t val)
	{
		/*
		 * Mask some of the bits of the config register because
		 * we don't want to change them by accident.
		 */
		val = val | 0x0FEC;

		uint8_t buf[2];
		buf[0] = val & 0xFF;
		buf[1] = (val >> 8) & 0xFF;

		writeEnable();
		Bus::write(WRITE_NV_CONFIG_REG, buf, 2);
		writeDisable();
		waitUntilReady();
	}

	void writeEnable()
	{
		if(!read_only_) {
			Bus::write(WRITE_ENABLE);
		}
	}

	void writeDisable()
	{
		Bus::write(WRITE_DISABLE);
	}

	static void addrToBuf(uint8_t* buf, uint32_t addr)
	{
		for(int i = 0; i < 4; i++)
		{
			buf[3 - i] = (addr >> 8*i) & 0xFF;
		}
	}

	template <class CS, class CLK>
	static void factoryResetSequence(int cycles)
	{
		CS::low();
		Thread::sleep(5);
		for(int i = 0; i < cycles; i++)
		{
			CLK::high();
			Thread::sleep(5);
			CLK::low();
			Thread::sleep(5);
		}
		CS::high();
		Thread::sleep(20);
	}

	static std::string logtag()
	{
	  return "FLASH_DRIVER";
	}

	enum Commands
	{
		READ_ID = 0x9F,

		WRITE_ENABLE = 0x06,
		WRITE_DISABLE = 0x04,

		READ_STATUS_REG = 0x05,
		WRITE_STATUS_REG = 0x01,

		READ_FLAG_STATUS_REG = 0x70,
		CLEAR_FLAG_STATUS_REG = 0x50,

		//non-volatile configuration register
		READ_NV_CONFIG_REG = 0xB5,
		WRITE_NV_CONFIG_REG = 0xB1,

		//volatile configuration register
		WRITE_VOL_CONFIG_REG = 0x85,
		READ_VOL_CONFIG_REG = 0x81,

		READ_EXT_ADDRESS_REG = 0xC8,
		WRITE_EXT_ADDRESS_REG = 0xC5,

		RESET_ENABLE = 0x66,
		RESET_MEMORY = 0x99,

		READ = 0x03,
		PROGRAM_PAGE = 0x02,

		SUBSECTOR_ERASE = 0x20,
		SECTOR_ERASE = 0xD8,
		DIE_ERASE = 0xC4
	};

	bool read_only_ = false;

	/**
	 * Flag status register bit definitions
	 */
	static const uint8_t FSR_ADDRESSING_MODE		= 0x01;
	static const uint8_t FSR_PROTECTION 			= 0x02;
	static const uint8_t FSR_PROGRAM_SUSPEND 		= 0x04;
	static const uint8_t FSR_VPP 					= 0x08;
	static const uint8_t FSR_PROGRAM 				= 0x10;
	static const uint8_t FSR_ERASE 					= 0x20;
	static const uint8_t FSR_ERASE_SUSPEND			= 0x40;
	static const uint8_t FSR_PRG_ERS_CTRL	 		= 0x80;
};

}
#endif /* FLASHDRIVER_H */
