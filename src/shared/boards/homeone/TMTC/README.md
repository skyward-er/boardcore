# TMTCManager
This module is in charge of managing the communication with the ground station through a Gamma868 driver,
handling the incoming Telecommands and buffering the packets to send.

| File  |  Description |
|-------|--------------|
| TMTCManager | Creates the sender, receiver and RF driver. Has a non-blocking send() function. |
| TMTCSender  | ActiveObject. Has an outBuffer that can be accessed from the Manager.Reads bytes from the buffer and frowards them to the driver. |
| TMTCReceiver | Parses the received messages into Mavlink messages and handles them depending on the type. |
| CircularBuffer | A fixed dimension synchronized array, where messages can be enqueued and dequeued without delays. |
| TMTC_config | Lists all the includes and defines that are used in the module. |