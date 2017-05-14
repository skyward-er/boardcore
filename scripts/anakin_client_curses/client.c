#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <ncurses.h>
#include <signal.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>

volatile int stop = 0;
uint8_t buf[8192] = {0};
uint16_t rptr = 0, wptr = 0;

#pragma pack(1)
struct vec3_t {
    float x;
    float y;
    float z; 
};
#pragma pack()

enum DataType
{
    DATA_VEC3  = 0,
    DATA_QUAT  = 1,
    DATA_FLOAT = 2,
    DATA_INT   = 3,
    DATA_STRING= 4,
    DATA_LIMITED_INT = 5,
    DATA_UINT32 = 6
};

const char* sensor_names[] = 
{
    "ACCEL_MPU9250",
    "ACCEL_INEMO",
    "ACCEL_MAX21105",

    "GYRO_MPU9250",
    "GYRO_INEMO",
    "GYRO_FXAS21002",
    "GYRO_MAX21105",

    "COMPASS_MPU9250",
    "COMPASS_INEMO",

    "TEMP_MPU9250",
    "TEMP_INEMO",
    "TEMP_LPS331AP",
    "TEMP_MAX21105",
    "TEMP_MS580",

    "PRESS_LPS331AP",
    "PRESS_MS580",

    "UNUSED_16",
    "DMA_TX_FIFOSZ",
    "DMA_RX_FIFOSZ",
    "DMA_FIFO_ERR",
    "CPU%",
    "LOG_QUEUE_SZ"
};

uint8_t sensor_type[] = 
{
    DATA_VEC3,
    DATA_VEC3,
    DATA_VEC3,

    DATA_VEC3,
    DATA_VEC3,
    DATA_VEC3,
    DATA_VEC3,

    DATA_VEC3,
    DATA_VEC3,

    DATA_FLOAT,
    DATA_FLOAT,
    DATA_FLOAT,
    DATA_FLOAT,
    DATA_FLOAT,

    DATA_FLOAT,
    DATA_FLOAT,

    DATA_INT,
    DATA_LIMITED_INT,
    DATA_LIMITED_INT,
    DATA_UINT32,
    DATA_UINT32,
    DATA_LIMITED_INT
};

#define NUM_SENSORS ((int)((sizeof(sensor_names) / sizeof(sensor_names[0]))))


void on_signal(int s)
{
    if(!stop)
    {
        signal(s, on_signal);
        stop = 1; 
    }
}

int logline = 0;
void log_string(const char* data, int len)
{
    if(len <= 0)
        return;

    move(NUM_SENSORS + 2 + logline, 10);
    attron(COLOR_PAIR(1));
    printw(data);
    if(len < 30)
        for(int i=len;i<30;i++)
            printw(" ");
    refresh();
    logline = (logline+1)%20;
}

void draw_sensor(int id, int type, const void* data)
{
    char tmp[256] = {0};

    if(id >= (int)NUM_SENSORS)
    {
        sprintf(tmp, "Got packet for invalid sensor %d\n", id);
        log_string(tmp, strlen(tmp)); 
        return;
    }

    if(type < 0 || type > 6 || type == 4)
    {
        sprintf(tmp, "Got packet for invalid type %d\n", type);
        log_string(tmp, strlen(tmp)); 
        return; 
    }

    move(1+id, 10);
    sprintf(tmp, "%20s : ", sensor_names[id]);

    uint8_t color = (type != 5) ? (type + 1) : 2;
    attron(COLOR_PAIR(color));
    printw(tmp);
    attroff(COLOR_PAIR(color));

    int all_null = 1;
    for(int i=0;i<NUM_SENSORS && all_null == 1;i++)
        if(((const char *)data)[i] != 0)
            all_null = 0;
    if((data == NULL || all_null == 1) && type != 5)
    {
        attron(COLOR_PAIR(5));
        attron(A_BOLD);
        printw("[NULL]                                  ");
        attroff(A_BOLD);
        attroff(COLOR_PAIR(5));
        return;
    }

    const uint8_t* i            = (const uint8_t*) data;
    const struct vec3_t* v  = (const struct vec3_t*) data;
    const float* f          = (const float*) data;
    const uint8_t* ui       = (const uint8_t*) data;
    const uint32_t* ui32       = (const uint32_t*) data;

    switch(type)
    {
        case DATA_FLOAT: 
            sprintf(tmp, "%7.4f                      ", *f);
            printw(tmp);
            break;
        case DATA_INT: 
            sprintf(tmp, "%d                         ", *i);
            printw(tmp);
            break;
        case DATA_UINT32: 
            sprintf(tmp, "%d                         ", *ui32);
            printw(tmp);
            break;
        case DATA_VEC3:
            sprintf(tmp, "(%7.4f, %7.4f, %7.4f)", v->x, v->y, v->z);
            printw(tmp);
            break;
        case DATA_LIMITED_INT:
        {
            uint32_t vv = (*ui * 27) / 256;
            for(uint32_t i=0;i<27;i++)
            {
                if(i < vv)
                    printw("#");
                else
                    printw(".");
            }
            break;
        }
        default:
            printw("[NOT IMPLEMENTED]");
            break;
    }
}

int sfd;
int init_serial(const char *device)
{
    sfd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if(sfd < 0)
    {
        printf("Cannot open ttyUSB0\n");
        return 1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if(tcgetattr(sfd, &tty) != 0)
    {
        printf("Cannot get attributes\n");
        return 1;
    }
    cfmakeraw(&tty);
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tcsetattr(sfd, TCSANOW, &tty);

    return 0;
}

void parse(int len)
{
    if(len == 0)
        return;

    uint8_t type = buf[0];
    if(type == DATA_STRING)
        log_string((const char *)&buf[1], len-1);
    else 
    {
        uint8_t id = buf[1];
        draw_sensor(id, type, &buf[2]);
    }
}

int fromHex(char l)
{
    if(l >= '0' && l <= '9')
        return l - '0';
    if(l >= 'a' && l <= 'f')
        return 10 + l - 'a';
    if(l >= 'A' && l <= 'F')
        return 10 + l - 'A';
    return 0;
}

int read_and_draw()
{
    memset(buf, 0, sizeof(buf));
    int bpos = 0;
    char b;
    char tmp = 0;
    do {
        read(sfd, &b, 1);
        if(b == '\n' || b == '\r')
            break;
        else {
            if(tmp == 0)
                tmp = b;
            else {
                buf[bpos++] = (fromHex(tmp) << 4) | fromHex(b);
                tmp = 0;
            }
        }
    } while(1);
    parse(bpos);
    return 0;
}

int main(int argc, char *argv[])
{
    if(argc < 2)
    {
        printf("Usage: %s <PORT>\n", argv[0]);
        return -1;
    }
    if(init_serial(argv[1]))
        return -1;

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    initscr();
    start_color();
    init_pair(1, COLOR_BLUE, COLOR_BLACK);
    init_pair(2, COLOR_GREEN, COLOR_BLACK);
    init_pair(3, COLOR_CYAN, COLOR_BLACK);
    init_pair(4, COLOR_WHITE, COLOR_BLACK);
    init_pair(5, COLOR_RED, COLOR_BLACK);
    init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
    erase();
    refresh();

    log_string("Ready...", 8);
    while(!stop)
    {
        if(read_and_draw() == 1)
            stop = 1;
        refresh();
    }

    endwin();

    signal(SIGINT, 0);
    signal(SIGTERM, 0);

    return 0;
}
