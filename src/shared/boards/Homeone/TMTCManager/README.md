# TMTCManager
This module is in charge of managing the communication with the ground station,
handling the incoming Telecommands and buffering the packets to send.
<br/> The module is designed to work with a Gamma868 transceiver and uses Mavlink
as communication protocol.

## Table of Contents

| File  |  Description |
|-------|--------------|
| TMTCManager | Creates the sender and receiver threads. Has a non-blocking enqueueMsg() that buffers the out messages.
The enqueued messages will be consumed by the sender thread and sent over the link.
The receiving thread waits for a message to be received and handles it with the appropriate MessageHandler. |
| MessageHandlers | Contains a handler for each type of message. The link between mavlink messages type and
its respective handler is stored in a map. |
| CircularBuffer | A fixed dimension synchronized array, where messages can be enqueued and dequeued without delays.
Used for storing messages before sending packets through the link. |
| TMTC_Config | Lists all the includes and defines that are used in the module. |

## Configuration

See TMTC_Config.h file to see all the configurable parameters.

### Use

Include TMTCManager.h and use `sTMTCManager` to access the object, such as:

```
sTMTCManager->enqueueMg(msg, msg_len);
```