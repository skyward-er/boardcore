/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Valerio Flamminii
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "qspi-flash.h"

qspi_flash mymemory;

int main()
{

    mymemory.init();

    std::vector<uint8_t> v;
    v.reserve(20000);
    for (uint32_t i = 0; i < v.capacity(); i++)
        v.push_back(5);
    v.resize(v.capacity());

    /*
    printf("vettore: \n");
    uint32_t i = 0;
    for (i = 0; i < v.size(); i++)
    {
        printf("v[%d]: %d \n", i, v[i]);
    }
    printf("vector size: %u\n", v.size());
    printf("vector capacity: %d\n", v.capacity());
    */

    // printf("result: %d\n", mymemory.write_vector(v, 1019, true));

    // --------- read FUNZIONA -------------
    std::vector<uint8_t> v2;
    mymemory.read_sector(v2, 1023);
    printf("\nvettore v2: \n");
    uint32_t a = 0;
    for (a = 0; a < FlashMemory::SECTOR_SIZE; a++)
    {
        printf("v2[%d]: %d \n", a, v2[a]);
    }
    printf("v2 size: %d\n", v2.size());
    printf("v2 capacity: %d\n", v2.capacity());

    // ------------- ATTENZIONE SU FUNZIONI DI LETTURA -----------------
    // numero di bytes della flash = 2 ^ (FSIZE + 1)
    // impostare la corretta dimensione della flash è necessario al corretto
    // funzionamento delle operazioni di lettura dalla flash. tutte le
    // operazioni di lettura della memoria a partire da un indirizzo che non può
    // essere contenuto nella memoria di dimensione indicata (FSIZE) non vengono
    // eseguite dalla periferica QUADSPI. esempio: se la dimensione impostata è
    // di 4 byte allora gli indirizzi accettati sono: 0-3b.

    // - read_sector funziona bene.

    // - program_sector funzionano bene con verify_write

    // - page_program funziona beneeeeee

    // - readID() funziona bene con sleep(1ms)

    // - test() funziona bene

    // - studiare come aggiungere quadspi mode: non vale la pena ad oggi

    // - waitProgress più veloce: funziona tutto ok

    // ----------- TEST ------ lettura security register
    // ho modificato byte_program e read_security_reg()
    // senza sleep() non ci sono dati nella FIFO. con sleep arriva sempre il
    // byte data. read_security_reg() sembra leggere il registro, da provare con
    // suspend commands. se non va, check con rilettura dei dati.

    // assumo che la lettura del security_register sia corretta, verifico la
    // riuscita delle operazioni più importanti rileggendo i dati.

    // su funzioni meno importanti controllare check_operation() e controllare
    // bene gli argomenti siano corretti. Nelle altre controllo con verify.

    // modifico byte_program con flag verify e controllo su argomenti: FUNZIONA
    // testata

    // modifico tutte le erase: FUNZIONANNO testate

    // modifico read_byte: FUNZIONA testata

    // page_program modificata: aggiunto verify flag FUNZIONA testata.

    // test finali page_program, read_sector, e program_sector FUNZIONANO
    // TESTATE.

    // METTERE FLAG INITIALISED 

    // - commentare meglio e sistemare funzioni public e private.

    // - forse aggiungere controllo anti loop operation.

    while (true)
        Thread::sleep(1000);
}