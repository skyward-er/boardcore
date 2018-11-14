#include <Common.h>
#include <fcntl.h>

#define LED_FREQ 200
#define PRINT_FREQ 500
#define MSG_LEN 28

using namespace miosix;

typedef miosix::Gpio<GPIOG_BASE, 13> redLed;
typedef miosix::Gpio<GPIOG_BASE, 14> greenLed;

int radio_fd;
int gps_fd;
static const char msg[MSG_LEN] = "May the force be with you.\n";

/**
 * Leds Thread loop
 */
static void* ledsThreadExecutor(void*) {

	while(1){

		// Red
		redLed::high();
		miosix::Thread::sleep(LED_FREQ);
		redLed::low();
		miosix::Thread::sleep(LED_FREQ);

		// Green
		greenLed::low();
		miosix::Thread::sleep(LED_FREQ);
		greenLed::high();
		miosix::Thread::sleep(LED_FREQ);

		// Both
		redLed::high();
		greenLed::high();
		miosix::Thread::sleep(LED_FREQ);
		redLed::low();
		greenLed::low();
		miosix::Thread::sleep(LED_FREQ);
	}

	printf("[leds] Loop ended: is it because your sister is gay?\n");
	return 0;
}

/**
 * Radio Thread loop
 */
static void* radioThreadExecutor(void*) {

	while(1){

	    write(radio_fd, msg, MSG_LEN);
		miosix::Thread::sleep(PRINT_FREQ);
	}

	printf("[radio] Radio loop ended: what have you done?\n");
	return 0;
}

/**
 * Gps Thread loop
 */
static void* gpsThreadExecutor(void*) {

	while(1) {

		write(gps_fd, msg, MSG_LEN);
		miosix::Thread::sleep(PRINT_FREQ);
	}

	printf("[gps] Gps loop ended: try with google maps.\n");
	return 0;
}

/**
 * If everything is working:
 * - Board LEDS should blink with the pattern: RED (PG13), GREEN(PG14), both.
 * - RADIO should send a message every 500ms on radio port 		serial3(PB9-PB10)@57600baud
 * - GPS should send a message every 500ms on gps serial 		serial2(PA2-PA3)@115200baud
 * - DEFAULT SERIAL should echo every char received 			serial1(PA9-PA10)@19200baud
 */
int main(){

	// Leds init
	redLed::mode(Mode::OUTPUT);
    greenLed::mode(Mode::OUTPUT);;
    Thread::create(ledsThreadExecutor, STACK_DEFAULT_FOR_PTHREAD, MAIN_PRIORITY);
    printf("[leds] Leds ok (too easy)\n");

	// Radio init
	radio_fd = open("/dev/radio", O_RDWR);
    if (radio_fd < 0){
        printf("[main] Cannot open radio device\n");
    }
    else{
    	Thread::create(radioThreadExecutor, STACK_DEFAULT_FOR_PTHREAD, MAIN_PRIORITY);
    	printf("[main] Radio OK\n");
    }

    // Gps init
    gps_fd = open("/dev/gps", O_RDWR);
    if (gps_fd < 0){
        printf("[main] Cannot open gps device\n");
    }
    else {
    	Thread::create(gpsThreadExecutor, STACK_DEFAULT_FOR_PTHREAD, MAIN_PRIORITY);
    	printf("[main] GPS OK\n");
    }

    // Main loop
    char c;
    while(1){
    	scanf("%c", &c);
    	printf("[main] did you say %c?\n", c);
    }

    printf("[main] Main loop ended: not nice.\n");

}