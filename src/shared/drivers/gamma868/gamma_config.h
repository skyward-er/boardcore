#ifndef CONFIG_H
#define CONFIG_H



#endif /* CONFIG_H */

//Object config
#define MAX_BUFFER 128

typedef Gpio<GPIOB_BASE,0> led;

struct Configuration{
    int local_addr[3] = {127, 127, 127};
    int dest_addr[3] = {127, 127, 127};
    int lora_mode = 6;
    int lora_pow = 15;
    int handshake = 0;
    int baudrate = 4;
};

//Protocol config
#define HEAD_LEN 2
#define CMD_LEN 1
#define DATA_LEN 64
#define END_LEN 1

#define START '#'
#define CMD '!'
#define DATA '?'
#define END  '%'

