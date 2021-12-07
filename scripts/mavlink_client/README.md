# Mavlink Client

These scripts can be used to transmit and receive mavlink data from a PC, either
connected via serial port to a transceiver or directly connected to one of the boards.

Use `make` to compile the 3 `.c` files:

* continuous_tx: sends a message of raw bytes with a given length at a certain
frequency. This was used to test the LoRa modules.
* mavlink_demo_tx: sends a message whenever it receives a char on stdin
* mavlink_demo_rx: receives and prints packets, eventually sending back an ACK.

`plot.py`: reads value on stdin and plots them. Can be used piped with the output
of mavlink_demo_rx.
