// qspi-flash driver TEST, aggiunto l' eseguibile in cmakelist
// dipendenze: aggiunto qspi-flash.cpp in cmake.boardcore 
// il test viene eseguito ma non sembra leggere l'ID

#include "qspi-flash.h"

qspi_flash mymemory;

int main() {


    // test readID() FUNZIONAAAA
    printf("starting ID test!\n");

    mymemory.init(); 
    
    uint32_t ID = mymemory.readID(); 

    printf("ID: %x\n", ID); 

    printf("end ID test\n"); 

    
    // test write enable command FUNZIONAAAAAAA  
    /*
    printf("start write_enable command\n"); 

    mymemory.write_enable(); 

    printf("----- end write_enable command\n"); 
    */
    
    // test write disable command FUNZIONAAAAAAA  
    /*  
    printf("start write_disable command\n"); 

    mymemory.write_disable();

    printf("----- end write_disable command\n");
    */

    // test funzione isInProgress 
    /*
    printf("test funzione isInProgress()\n"); 
    
    printf("valore isInProgress(): %d\n", mymemory.isInProgress());

    printf("----- end isinProgress test\n");

    // test read security_reg()
    printf("\nvalore security register: %d\n", mymemory.read_security_reg());
    */ 

    // test funzione sector erase SEMBRA FUNZIONARE FINO AD ORA !!!!
    /*
    printf("\nstart TEST sector_erase()\n"); 

    printf("valore ritornato: %d\n", mymemory.sector_erase(0));

    printf("end test sector_erase()\n");  
    */

    // test funzione byte_program() 
    printf("\ntest funzione byte_program()\n"); 

    printf("valore ritornato: %d\n", mymemory.byte_program(55U, 0)); 

    printf("end test byte_program()\n");

    // test funzione read_byte(): OCCHIO IMPOSTA SEMPRE LA DIMENSIONE DELLA FLASH 
    printf("\ntest funzione read_byte()\n");

    // test block32_erase(): FUNZIONA 
    printf("valore block32_erase(): %d\n", mymemory.block32_erase(0x3FFFFF)); 

    // test block64_erase(): FUNZIONA 
    printf("valore block64_erase(): %d\n", mymemory.block64_erase(0x3fffff)); 
    
    printf("valore read_byte: %d\n", mymemory.read_byte(0));

    printf("----- end read_byte() test\n"); 

    // test funzione reset(): FUNZIONA 
    /*
    printf("\n------- start test funzione reset() ----------\n"); 

    Thread::sleep(100); 
    mymemory.write_enable(); 
    printf("status reg prima : %d\n", mymemory.read_status_reg()); 
    //mymemory.software_reset(); 
    printf("status reg dopo: %d\n", mymemory.read_status_reg()); 

    printf("------- end test funzione reset() ----------\n"); 
    */


    // test erase chip function(): FUNZIONA
    /*
    printf("\nstart TEST erase chip function\n"); 

    printf("valore ritornato: %d\n", mymemory.chip_erase()); 

    printf("end of the erase chip operation.\n "); 
    */

    // ------------- ATTENZIONE SU FUNZIONI DI LETTURA ----------------- 
    // numero di bytes della flash = 2 ^ (FSIZE + 1)
    // impostare la corretta dimensione della flash è necessario al corretto funzionamento 
    // delle operazioni di lettura dalla flash.
    // tutte le operazioni di lettura della memoria a partire da un indirizzo che non può 
    // essere contenuto nella memoria di dimensione indicata (FSIZE) non vengono eseguite 
    // dalla periferica QUADSPI. 
    // esempio: 
    // se la dimensione impostata è di 4 byte allora gli indirizzi accettati sono: 0-3b


    while (true)
        Thread::sleep(1000);
}