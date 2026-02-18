#include <drivers/MCP23S17/MCP23S17.h>
#include <drivers/MCP23S17/MCP23S17Defs.h>
#include <miosix.h>
#include <iostream>

using namespace miosix;
using namespace Boardcore;


int main() {

    // Pin config
    GpioPin cs(GPIOA_BASE, 0);
    GpioPin sck(GPIOB_BASE, 3);
    GpioPin miso(GPIOB_BASE, 4);
    GpioPin mosi(GPIOD_BASE, 6);

    cs.mode(Mode::OUTPUT);
    cs.high();

    sck.alternateFunction(5);
    sck.mode(Mode::ALTERNATE);

    miso.alternateFunction(5);
    miso.mode(Mode::ALTERNATE);

    mosi.alternateFunction(5);
    mosi.mode(Mode::ALTERNATE);

    SPIBus spi(SPI3);

    MCP23S17 mcp(spi, cs);


    std::cout << "TESTING MCP23S17\n";

    std::cout << "\n################## TESTING INIT SEQUENCE ##################\n";

    mcp.init();
    std::cout << mcp.readRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::IODIRA)) << '\n';
    std::cout << mcp.readRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::IODIRB)) << '\n';
    std::cout << mcp.readRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPPUA)) << '\n';
    std::cout << mcp.readRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPPUB)) << '\n';
    std::cout << mcp.readRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPINTENA)) << '\n';
    std::cout << mcp.readRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPINTENB)) << '\n';
    std::cout << mcp.readRegister(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::DEFVALA)) << '\n';
    std::cout << mcp.readRegister(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::DEFVALB)) << '\n';
    std::cout << mcp.readRegister(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::INTCONA)) << '\n';
    std::cout << mcp.readRegister(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::INTCONB)) << '\n';

    std::cout << "\n################## TESTING RANDOM W/R SEQUENCES - GPIO REGISTERS ##################\n";

    // IODIR

    mcp.writeRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::IODIRA), 255);
    std::cout << mcp.readRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::IODIRA)) << '\n';

    mcp.writeRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::IODIRB), 255);
    std::cout << mcp.readRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::IODIRB)) << '\n';

    // IOPOL

    mcp.writeRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::IOPOLA), 255);
    std::cout << mcp.readRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::IOPOLA)) << '\n';

    mcp.writeRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::IOPOLB), 255);
    std::cout << mcp.readRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::IOPOLB)) << '\n';

    // GPINTEN

    mcp.writeRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPINTENA), 255);
    std::cout << mcp.readRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPINTENA)) << '\n';

    mcp.writeRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPINTENB), 255);
    std::cout << mcp.readRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPINTENB)) << '\n';

    // GPPU

    mcp.writeRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPPUA), 255);
    std::cout << mcp.readRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPPUA)) << '\n';

    mcp.writeRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPPUB), 255);
    std::cout << mcp.readRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPPUB)) << '\n';

    // GPIO*_EXT

    mcp.writeRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPIOA_EXT), 255);
    std::cout << mcp.readRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPIOA_EXT)) << '\n';

    mcp.writeRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPIOB_EXT), 255);
    std::cout << mcp.readRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPIOB_EXT)) << '\n';

    // OLAT

    mcp.writeRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::OLATA), 255);
    std::cout << mcp.readRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::OLATA)) << '\n';

    mcp.writeRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::OLATB), 255);
    std::cout << mcp.readRegister(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::OLATB)) << '\n';


    std::cout << "\n################## TESTING RANDOM W/R SEQUENCES - CTRL REGISTERS ##################\n";

    // DEFVAL

    mcp.writeRegister(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::DEFVALA), 255);
    std::cout << mcp.readRegister(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::DEFVALA)) << '\n';

    mcp.writeRegister(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::DEFVALB), 255);
    std::cout << mcp.readRegister(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::DEFVALB)) << '\n';

    // INTCON

    mcp.writeRegister(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::INTCONA), 255);
    std::cout << mcp.readRegister(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::INTCONA)) << '\n';

    mcp.writeRegister(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::INTCONB), 255);
    std::cout << mcp.readRegister(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::INTCONB)) << '\n';



    std::cout << "\n################## TESTING RANDOM W/R SEQUENCES - IOCON REGISTER ##################\n";

    // Set all bits to one using the single methods, then read each single pin, afterwards repeat the procedure putting all bits to 0
    mcp.setBANK(1);
    mcp.setMIRROR(1);
    mcp.setSEQOP(1);
    mcp.setDISSLW(1);
    mcp.setHAEN(1);
    mcp.setODR(1);
    mcp.setINTPOL(1);

    std::cout << mcp.readBit(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::IOCON), 7) << '\n';
    std::cout << mcp.readBit(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::IOCON), 6) << '\n';
    std::cout << mcp.readBit(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::IOCON), 5) << '\n';
    std::cout << mcp.readBit(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::IOCON), 4) << '\n';
    std::cout << mcp.readBit(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::IOCON), 3) << '\n';
    std::cout << mcp.readBit(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::IOCON), 2) << '\n';
    std::cout << mcp.readBit(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::IOCON), 1) << '\n';

    mcp.setBANK(0);
    mcp.setMIRROR(0);
    mcp.setSEQOP(0);
    mcp.setDISSLW(0);
    mcp.setHAEN(0);
    mcp.setODR(0);
    mcp.setINTPOL(0);

    std::cout << mcp.readBit(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::IOCON), 7) << '\n';
    std::cout << mcp.readBit(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::IOCON), 6) << '\n';
    std::cout << mcp.readBit(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::IOCON), 5) << '\n';
    std::cout << mcp.readBit(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::IOCON), 4) << '\n';
    std::cout << mcp.readBit(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::IOCON), 3) << '\n';
    std::cout << mcp.readBit(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::IOCON), 2) << '\n';
    std::cout << mcp.readBit(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::IOCON), 1) << '\n';


    // Clean config

    mcp.init();


    std::cout << "\n################## TESTING SPECIFIC METHODS ##################\n";

    // Pin I/O (IODIR)

    mcp.setPinOut_A(0);
    std::cout << mcp.readBit(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::IODIRA), 0) << '\n';

    mcp.setPinOut_B(0);
    std::cout << mcp.readBit(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::IODIRA), 0) << '\n';

    mcp.setPinIn_A(0);
    std::cout << mcp.readBit(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::IODIRA), 0) << '\n';

    mcp.setPinIn_B(0);
    std::cout << mcp.readBit(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::IODIRB), 0) << '\n';

    // Pin POLARITY (IOPOL)

    mcp.setPinPolarity_A(0, 0);
    std::cout << mcp.readBit(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::IOPOLA), 0) << '\n';

    mcp.setPinPolarity_B(0, 0);
    std::cout << mcp.readBit(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::IOPOLB), 0) << '\n';

    mcp.setPinPolarity_A(0, 1);
    std::cout << mcp.readBit(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::IOPOLA), 0) << '\n';

    mcp.setPinPolarity_B(0, 1);
    std::cout << mcp.readBit(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::IOPOLB), 0) << '\n';

    // Interrupt on change (GPINTEN)

    mcp.enableInterruptOnChange_A(0);
    std::cout << mcp.readBit(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPINTENA), 0) << '\n';

    mcp.enableInterruptOnChange_B(0);
    std::cout << mcp.readBit(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPINTENB), 0) << '\n';

    mcp.disableInterruptOnChange_A(0);
    std::cout << mcp.readBit(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPINTENA), 0) << '\n';

    mcp.disableInterruptOnChange_B(0);
    std::cout << mcp.readBit(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPINTENB), 0) << '\n';

    // Pull up (GPPU)

    mcp.enablePullUp_A(0);
    std::cout << mcp.readBit(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPPUA), 0) << '\n';

    mcp.enablePullUp_B(0);
    std::cout << mcp.readBit(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPPUB), 0) << '\n';

    mcp.disablePullUp_A(0);
    std::cout << mcp.readBit(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPPUA), 0) << '\n';

    mcp.disablePullUp_B(0);
    std::cout << mcp.readBit(mcp.getGpioAddress(MCP23S17Defs::GPIO_REG::GPPUB), 0) << '\n';

    // Default value (DEFVAL)
    
    mcp.setDefaultValue_A(0, 1);
    std::cout << mcp.readBit(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::DEFVALA), 0) << '\n';

    mcp.setDefaultValue_B(0, 1);
    std::cout << mcp.readBit(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::DEFVALB), 0) << '\n';
    
    // Interrupt comparison (INTCON)

    mcp.setInterruptComparison_A(0, 1);
    std::cout << mcp.readBit(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::INTCONA), 0) << '\n';

    mcp.setInterruptComparison_B(0, 1);
    std::cout << mcp.readBit(mcp.getCtrlAddress(MCP23S17Defs::CTRL_REG::INTCONB), 0) << '\n';

    // Pins and Latches (GPIO*_EXT, OLAT)

    mcp.writePin_A(0, 0);
    std::cout << mcp.readPin_A(0) << '\n';

    mcp.writePin_B(0, 0);
    std::cout << mcp.readPin_B(0) << '\n';
    
    mcp.writeLatch_A(0, 1);
    std::cout << mcp.readLatch_A(0) << '\n'; 
    std::cout << mcp.readLatch_A(0) << '\n'; 

    mcp.writeLatch_B(0, 1);
    std::cout << mcp.readLatch_B(0) << '\n'; 
    std::cout << mcp.readLatch_B(0) << '\n'; 


    return 0;
}