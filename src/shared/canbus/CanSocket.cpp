/* CAN-Bus Driver
 *
 * Copyright (c) 2015 Skyward Experimental Rocketry
 * Authors: Matteo Piazzolla, Alain Carlucci
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
TODO: 
- gli id devono diventare uint16_t
- receive non bloccante
*/

#include "CanSocket.h"
#include "CanBus.h"

CanSocket::CanSocket(const uint16_t filter_id) : filter_id(filter_id) {
    pthread_mutex_init(&mutex,NULL); 
    pthread_cond_init(&cond,NULL);
}

/**
    Apre una connessione simil-socket tramite un oggetto CanBus 
    \param bus L'istanza del CanBus
    \param id l'id Standard o Esteso del socket 
    TODO: rimuoverlo da parametro e prenderlo direttamente dall'oggetto
*/
void CanSocket::open(CanBus *bus){
    if(isOpen()) 
        close();
    bus->registerSocket(this);
    this->bus=bus;
}

/**
    prende dalla coda un messaggio ricevuto
    \param message dove copiare il messaggio
    \param size dove scrivere la lunghezza del messaggio ricevuto
*/
bool CanSocket::receive(void *message, int& size){
    if(!isOpen())
        return false;

    pthread_mutex_lock(&mutex);
    while(receivedMessageQueue.empty())
        pthread_cond_wait(&cond,&mutex);

    size= receivedMessageQueue.front().second;
    memcpy(message, receivedMessageQueue.front().first, size );
    delete receivedMessageQueue.front().first;
    receivedMessageQueue.pop_front();
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&mutex);

    return true;
}

/**
    aggiunge un messaggio alla coda interna
*/
void CanSocket::addToMessageList(unsigned char *message, uint8_t size){
    pthread_mutex_lock(&mutex);
    uint8_t* msg = new uint8_t[size+1]; //TODO chiedere a fede
    memcpy(msg, message, size+1);
    msg_p toPush (msg,size);
    receivedMessageQueue.push_back(toPush);
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&mutex);
}

/**
    chiude il socket rimuovendolo dal bus
*/
void CanSocket::close(){
    if(!isOpen()) 
        return;

    pthread_mutex_lock(&mutex);
    bus->unregisterSocket(this);
    bus=0;
    pthread_mutex_unlock(&mutex);

    pthread_mutex_lock(&mutex);
    for (list<msg_p>::iterator it = receivedMessageQueue.begin(); 
            it != receivedMessageQueue.end(); ++it){
        delete receivedMessageQueue.front().first;
        it = receivedMessageQueue.erase(it);
    }

    receivedMessageQueue.clear();
    pthread_mutex_unlock(&mutex);
}

CanSocket::~CanSocket(){
    close();
    pthread_mutex_destroy(&mutex);
    pthread_cond_destroy(&cond);
}

