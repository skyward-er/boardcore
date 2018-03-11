/*
 * ---------------------------
 *           PHASE 1
 * ---------------------------
 *
 * - EventScheduler
 *     - Start Motor
 *     - Apogee Timeout (nosecone ejection)
 *		- Sensor Samplers 
 *		- Apogee Recognition
 *
 *  - SDWriter
 *
 *  - DumbLogger 
 *
 * - HomeoneBoard:
 *     - Pressure
 *     - IMU
 *     - GPS
 *     - Thermal cutter 
 *     - Lauch detection
 * 
 */

 #include "HomeoneBoard.h"	
 #include <Common.h>
 
 using namespace miosix;
 using namespace std;
 
 // Scheduler events
 int testBoardEvent();
 int startMotorEvent();
 int apogeeTimeoutEvent();
 
 // Scheduler routines
 int launchDetectionRoutine();
 int apogeeRecognitionRoutine();
 int sendStateRoutine();

 
int main(){
	
	printf("\n");
	
	//sLog e sBoard sono macro statiche definite rispettivamente in 
    //	DumbLogger e HomeoneBoard che invocano i getInstance 
	// della classe corrispondente
	
	//In particolare, sBoard è definita alla fine di HomeoneBoard.h, che importa
	// DumbLogger.h dove è definita sLog
	
    sLog->logString("Initialized");
    sBoard->init();
	
	//Add events to scheduler 
	
	//Add launch detection routine to scheduler
	//lauch routine will then cause apogee recognition to be scheduled
	
	while(1)
	{
		
		// Receive external commands (?)
		
		printf("Still alive\n");
		Thread::sleep(1000);
	}
	
	return 0;
}
