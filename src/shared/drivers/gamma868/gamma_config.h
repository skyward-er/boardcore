#ifndef CONFIG_H
#define CONFIG_H



#endif /* CONFIG_H */

typedef Gpio<GPIOB_BASE,0> gammaLed;
typedef Gpio<GPIOB_BASE,2> gammaSwitch;

struct Configuration{
    int local_addr[3] = {126, 126, 126};
    int dest_addr[3] = {126, 126, 126};
    int lora_mode = 1;      //SF6
    int lora_pow = 15;      //+20dbm
    int handshake = 0;      //No handshake
    int baudrate = 0;       //9600 baud
};

//Object config
#define MAX_BUFFER 128
#define MIN_TO_SEND 1

//Protocol config
#define HEAD_LEN 2
#define CMD_LEN 1
#define DATA_LEN 64
#define END_LEN 1

#define START '#'
#define CMD '!'
#define DATA '?'
#define END  '%'

