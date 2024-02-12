// qspi-flash driver TEST, aggiunto l' eseguibile in cmakelist
// dipendenze: aggiunto qspi-flash.cpp in cmake.boardcore 
// il test viene eseguito ma non sembra leggere l'ID

#include "qspi-flash.h"

qspi_flash mymemory;

int main() {


    // test readID() 
    printf("starting!\n"); 

    uint32_t ID = mymemory.readID();

    printf("ID: %x\n", ID);

    printf("end ID test\n");

    
    // test write enable command
    printf("start write_enable command\n"); 
    // TODO: COMMITTTTT E POI FARE BENE FUNZIONE INIT QUADSPI E SOPRATTUTTO CHIAMARE INIT 
    // QUADSPI AL MOMENTO DELLA CREAZIONE DI UN' ISTASNZA DELLA CLASSE QUADSPI-FLASH NEL
    // DENTRO IL COSTRUTTORE. MI RACCOMANDO FARE ABORT PRIMA DI FARE ALTRE OPERAZIONI. 
    mymemory.write_enable();

    printf("end write_enable command\n");     

    while (true)
        Thread::sleep(1000);
}