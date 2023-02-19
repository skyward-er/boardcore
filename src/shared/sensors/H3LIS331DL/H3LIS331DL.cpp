/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Radu Raul
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

#include <sensors/H3LIS331DL/H3LIS331DL.h>

bool Boardcore::H3LIS331DL::init(){
  if (this->initialized) {
    lastError = SensorErrors::ALREADY_INIT;
    return false;
  }
  SPITransaction spiTr(this->spi);

  uint8_t whoami = spiTr.readRegister(Registers.WHO_AM_I);

  if (whoami != WHO_AM_I_VAL) {
      lastError = SensorErrors::INVALID_WHOAMI;
      return false;
  }

  uint8_t ctrlReg1 = 0b0000'0000; // Default: poweroff
  uint8_t powerMode = 0; // Remain in poweroff
  uint8_t dr = 0x0; // Data rate is 0 in case of low power mode

  if (this->pm == LOW_PWR) {
    if (this->odr > ODR_LP_10) {
      lastError = SensorErrors::INIT_FAIL;
      return false;
    }

    // the low power mode output data rate is set in PM bits in the CTRL_REG1 instead in the DR bytes
    // and it starts at configuration 010.
    // Configuration 000 and 001 are used (in order) as poweroff and normal power mode.
    powerMode = b010 + this->odr;
  } else {
    if (this->odr < ODR_50) {
      lastError = SensorErrors::INIT_FAIL;
      return false;
    }

    powerMode = 0b001; // powermode is set to normal.

    // As ODR_50 (and the following) does not start at 0 I need to subtract ODR_50 to align everything to 0
    dr = this->odr - ODR_50;
  }

  SETBITS(ctrlReg1, /* left shift */ 5, /*bitmask*/ 0b1110'0000, powerMode);
  SETBITS(ctrlReg1, 3, 0b0001'1000, dr);
  SETBITS(ctrlReg1, 0, 0b0000'0111, 0b111);

  spTr.writeRegister(Registers.CTRL_REG1, ctrlReg1);
  this->initialized = true;
}


H3LIS331DLData Boardcore::H3LIS331DL::sampleImpl(){
  // Timestamp of the last sample
  uint64_t lastSampleTimestamp;

  uint8_t buf[6] = {0, 0, 0, 0, 0, 0};

  // Read output data registers (X, Y, Z)
  {
      SPITransaction spiTr(spislave);
      spiTr.readRegisters(Registers.OUT_X | 0x40, buf, 6);
  }

  {
      // Disable interrupts and copy the last sample locally, as a new
      // interrupt may come just as we are reading it and causing a
      // race condition.
      miosix::FastInterruptDisableLock dLock;
      lastSampleTimestamp = lastInterruptTimestamp;
  }

  int16_t x = buf[0] | buf[1] << 8;
  int16_t y = buf[2] | buf[3] << 8;
  int16_t z = buf[4] | buf[5] << 8;

  return H3LIS331DLData(lastSampleTimestamp, x, y, z);
}

Boardcore::H3LIS331DL::H3LIS331DL(
  SPIBusInterface& spiBus,
  SPIBusConfig cfg,
  miosix::GpioPin cs,
  OutputDataRate odr,
  BlockDataUpdate bdu
) : spi(bus, cs, cfg), odr(odr), bdu(bdu), initialized(false) {}

Boardcore::H3LIS331DL::H3LIS331DL(
  SPIBusInterface& spiBus,
  miosix::GpioPin cs,
  OutputDataRate odr,
  BlockDataUpdate bdu
) : H3LIS331DL(spiBus, {}, cs, odr, bdu) {}
