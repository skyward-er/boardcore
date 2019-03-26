#Mavlink driver

The **MavChannel** object offers an interface to send and receive from a Transceiver object using an implementation of the Mavlink protocol. See `tests/test-mavlink.cpp` for an axample of usage.

This object includes only the basic protocol headers (`protocol.h`). To use this driver, you should include YOUR OWN implementation of the messages definition (`mavlink.h`). To create your implementation you can use Skyward's *Mavlink_editor*.

The `multi` folder contains an old implementation, with multiple senders and receivers which are not syncrhonized. The use of this driver is deprecated.