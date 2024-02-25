# Hardware in the loop framework

This README refers to the OBSW (On-Board Software) and matlab framework used for the **Hardware In the Loop** (HIL) testing.

## HIL framework - OBSW side

### Dependencies:
- skyward-boardcore branch: sensors-dev
- miosix branch: gcc-9.2.0

### Files:
- `test-SerialInterface.cpp`: Used to make quick tests of the serial communication with the simulator. It's a pretty simple example of usage of the *SerialInterface* module, used to test if the the serial connection is working properly.
- `test-hil.cpp`: Used to test the control algorithms.
  - enable timestamp timer for the HILsensors
  - define a *HILTransceiver* object
  - define all the sensors and the actuators used by the simulation
  - define the control algorithm
  - add the algorithms to the *addNotifyToBegin* queue of the *HILTransceiver* object
  - initialize all the sensors and actuators
  - start the HILTransceiver thread
  - sample the sensors and update the algorithm at the right rate
- `HILSimulationConfig.h`: Configuration file for the entrypoint; for every type of simulation you should create a directory with your entrypoint and one of this configuration file with these main informations:
  - `SIM_BAUDRATE`: Baudrate of the connection for the simulation
  - `SIM_PERIOD`: Time of each simulated step [in milliseconds]
  - `SENSOR_FREQ`: Frequency of every sensor you will use in the simulation [in Hz = samples/second]
  - `SensorData`: Data structure sent by the simulator
  - `ActuatorData`: Data structure that the simulator expects back
- `HILConfig.h`: In this file you have to set the right config file to include when a component need the structures or other configuration parameters. You have to choose a *Flag* that represents the config file.
e.g.
```
#if defined(HIL_SERIALINTERFACE)
#include "test-SerialInterface/HILSimulationConfig.h"
```
- `SerialInterface.h`: Opens a serial port on the OBSW. After his initialization it's ready to send and receive the data. The default baudrate value is 256000 by default. The default port number value is 2 (for the stm32f407vg_stm32f4discovery is tx_board=PA2  rx_board=PA3)
- `HILTransceiver.h`: Is an ActiveObject and provides an easier interface with the simulator automatically creating a SerialTestSensor object (you can pass the baudrate or the USART port to the constructor) and offers these methods:
  - `getSensorData` returns a pointer to the last data simulated. The struct returned (*SensorData*) reflects in number of elements, type and positions the struct sent from the simulator (the struct sent from matlab and SensorData should always correspond!).
  - `setActuatorData` sends the data to the simulator
  - `addNotifyToBegin` takes an *Algorithm* object as input and starts it when the first packet from the simulator is received
  - `addResetSampleCounter` takes a *HILSensor* as input (or every object that implements *HILTimestampManagement*) and notifies to it that new data has arrived from the simulator
- `MockAirbrakeAlgorithm.h`: Example of control algorithm. Receives in the constructor a *HILSensor* (or a kalman that implements *HILSensor* as in this case) and a *ServoInterface*. The algorithm have to *getLastSample* from the sensor passed, elaborates it and then sends the output to the actuator calling his *set* method
- `HILSensor`: Interface implemented by all the sensors and kalmans. To the constructor (templated with the type of the struct used by the sensor to store the last sample) we have to pass the reference to the *HILTransceiver* object, the *simulation period* and the *number of simulated samples* by matlab of this sensor (corresponding to the number of rows of the corresponding field of the sensorData structure in the matlab simulatos). They have to be initialized before use. Everytime the `sample` method is invoked, the sensor takes the last unread sample from the data simulated and creates a timestamp for that sample. If we sample more data then the available one we [continue receiving the last sample/throw an exception/print an error message on the stdin]. We have to register the sensor to the `addResetSampleCounter` queue of *HILTransceiver* in order to be notified when fresh new data is arrived from the simulator, so that the sensor can start reading the data from the beginning of the array. The method `getLastSample` returns the *HILSensorData* structure with the last sample.
- `HILServo`: Is a *ServoInterface* implementation. Interfaces the control algorithm to the HILTransceiver object. Invoking the method `set`, the value passed is converted (if needed) and then sent to the simulator.

### Usage of the hardware-in-the-loop framework:

To use this module as-is:
- The first thing to do is to develop the control algorithm in the `step` method of *MockAirbrakeAlgorithm*. Then you should check that the *HILServo* sends back to the simulator the data in the conversion you want.
- Now you should create the entrypoint in the `sbs.conf` file. In the *Defines:* field you should add the flag of your entrypoint corresponding to the one used in `HILConfig.h` in order to choose the right config file (e.g. `Defines: -DHIL`)
- After this you should be able to build the `test-hil.cpp` and flash the executable on the board.
- Then you should connect via serial the board to the computer (with a serial adapter). The connections between adapter and board are:
  - adapter_GND to board_GND
  - adapter_RX to board_TX (check table or cheatsheet for available serialPorts and relative pins)
  - adapter_TX to board_RX (check table or cheatsheet for available serialPorts and relative pins)

<center>

| **board**                    | **serialPort** | **board_TX** | **board_RX** |
| ---------------------------- | -------------- | ------------ | ------------ |
| stm32f429zi_stm32f4discovery | (USART)3       | PB10         | PB11         |
| stm32f407vg_stm32f4discovery | (USART)2       | PA2          | PA3          |

</center>

Now the board is ready for the simulation.

### Example of usage of the hardware-in-the-loop framework on the OBSW:

A basic example is available in the *src/tests/test-hil* directory.

## HIL framework - Matlab side

### Files:
- `serialib.h` and `serialib.cpp`: used to build the C++ code for matlab
- `serialbridge.cpp`: the file that implements the serialbridge feature in C++ (not necessary if you don't have to change the framework)
- `build.m`: used for building the serialbridge.cpp file
- `serialbridge.mexw64`: the compiled mex file that provides the serialbridge
    function for matlab (this is the only file needed for the simulation)
- `structToSingles.m`: function that converts a generic struct (with also nested structs) to an array of singles, ready to be sent on the serial communication

### Usage of the hardware-in-the-loop framework:

First of all, in order to execute the following commands and receive the data back the board should already be flashed and connected properly as above. Then the following functions can be used to program the hardware-in-the-loop simulator:
- `serialbridge("Open", string_serialPort, uint_baudrate)` opens the serial communication with the board; usually the serial communication is opened at the beginning of `start_simulation.m`:
  - **"Open"**: specifies we want to open the serial port
  - *string_serialPort*: is the serial port we want to open (eg: "COM6"); In order to find the right port you can use on Windows the command `mode` and see the name of the serial port with baudrate *19200* or *256000*.
  - *uint_baudrate*: is the baudrate of the port (eg: 256000)
- `serialbridge("Write", singleArray_Data)` sends the data to the board; *you should send all the data in one single write command*:
  - **"Write"**: specifies that we want to write on the serial port to the board
  - *singleArray_Data*: is the array of singles we want to write on serial (eg: [1 2 3 4.5 5.4]). If we want to send a struct (or even nested structs) we can use the function `structToSingles` to turn the struct into an array
- `singleArray_Data = serialbridge("Read", uint_nData)` to receive data from the board; matlab will wait on the read till all the data is received:
  - **"Read"**: specifies that we want to read from the serial port
  - *uint_nData*: How many floats to read from serial (eg: 1)
  - *singleArray_Data*: array of floats read from the serial (eg: alpha_degree)
- `serialbridge("Close")` closes the serial communication:
  - **"Close"**: specifies we want to close the serial port

When the simulator is ready and the board is flashed and connected you can execute the simulation as a normal project in matlab.

### Example of usage of the hardware-in-the-loop framework in matlab:
```
serialbridge("Open", "COM6", 256000); % Opens the serial port
serialbridge("Write", [1 2 3 4]);     % Sends the array "[1 2 3 4]" to the serial device
data = serialbridge("Read", 2);       % Receives 2 floats and stores them in the variable "data"
serialbridge("Close");                % Closes the serial port
```

### WARNING:
- It's possible to open just ONE serial port with serialbridge
- The files serialib.h and serialib.cpp were modified in order to fix compiling errors on windows
- the function `structToSingles` ignores the struct fields called *time* because the obsw forges it's own timestamps for every sample

## FAQ
Some common problems found using the framework:
- For every problem you encounter, first time reboot your board. If it doesn't work clear and flash the board again. also you can try to disconnect and reconnect the serial adapter.
- If matlab gives back an error about something wrong with array indices check the data received from the OBSW, if they exceed the limit of that value (for example giving to the airbrake aperture a value of 2000 degrees) matlab returns this error
- If you have problems on linux (on the matlab side) check if you are opening the right port and at the right baudrate; check also if the baudrate you are using is supported by your operative system
- If you are reading malformed data check the baudrate on OBSW and matlab side. they have to be the same
- If you are not able to read data and matlab stays in the *busy* state (to restart matlab usage you must reboot matlab, *ctrl+c* or *ctrl+z* doesn't work cause you are waiting in the mex file and matlab can't communicate to it... thanks matlab):
  - check if you connected the right pins
  - check if the USART you are using is free from any other communication (use the cheatsheet, I'm sorry)
  - try to use another USART (if you are lucky and it works please update the table with all the boards, USART and pins verified to work properly)
  - check if it's a problem on the obsw (use a simpler obsw test in order to check communication)
