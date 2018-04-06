# TMTCManager
This module is in charge of managing the communication with the ground station through a Gamma868 driver,
handling the incoming Telecommands and buffering the packets to send.

### TABLE of CONTENT

| File  |  Description |
|-------|--------------|
| TMTCManager | Creates the sender and receiver threads. Has a non-blocking enqueueMsg() function to send messages. |
| TCHandler | Contains a handler for each type of mavlink message |
| CircularBuffer | A fixed dimension synchronized array, where messages can be enqueued and dequeued without delays. |
| TMTCConfig | Lists all the includes and defines that are used in the module. |

### CONFIGURATION

See TMTCConfig.h file to see all the configurable parameters.

### USE

Include TMTCManager.h and use `sTMTCManager` to access the object.