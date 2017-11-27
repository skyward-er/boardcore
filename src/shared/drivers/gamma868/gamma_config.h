#ifndef CONFIG_H
#define CONFIG_H

// DISCOVERY gpio configuration
typedef Gpio<GPIOB_BASE, 0> gammaLed;
typedef Gpio<GPIOB_BASE, 2> gammaSwitch;

// Module internal config
struct Configuration
{
    int local_addr[3] = {126, 126, 126};
    int dest_addr[3]  = {126, 126, 126};
    int lora_mode     = 1;   // SF6
    int lora_pow      = 15;  //+20dbm
    int handshake     = 0;   // No handshake
    int baudrate      = 0;   // 9600 baud
};

// Buffer config
#define OUT_BUFFER_SIZE 10

/*
 * Change this if you want to send even if the buffer contains
 * less chars than the packet (note that the packet has fixed size).
 */
#define MIN_TO_SEND 5

#define SEND_SLEEP_TIME 0

// Protocol config
#define HEAD_LEN 2
#define CMD_LEN 1
#define DATA_LEN 5
#define END_LEN 1

#define START '#'
#define CMD '!'
#define DATA '?'
#define END '%'

#endif