#include "drivers/canbus/CanBus.h"
#include "drivers/canbus/CanManager.h"
#include "drivers/canbus/CanSocket.h"
#include "drivers/canbus/CanPublisher.h"
#include "drivers/canbus/CanUtils.h"

#include <Common.h>
#include <drivers/BusTemplate.h>

#ifndef CAN_ABSTRACTION_H
#define CAN_ABSTRACTION_H



enum LinkType{
    LINK_IGNITION,
    LINK_DEPLOY
};


class CanAbstraction{

public:
    const uint8_t BUS_ID=0;
    CanManager* can_manager;

    CanAbstraction(LinkType link){
        if(link == LINK_IGNITION){
            can_manager = new CanManager(CAN1);
            canbus_init_t st0 = {
                CAN1, miosix::Mode::ALTERNATE, 9, {CAN1_RX0_IRQn, CAN1_RX1_IRQn}};
            can_manager->addBus<GPIOA_BASE, 11, 12>(st0);
        }
        else{
            can_manager = new CanManager(CAN2);
            canbus_init_t st1 = {
                CAN2, miosix::Mode::ALTERNATE, 9, {CAN2_RX0_IRQn, CAN2_RX1_IRQn}};
            can_manager->addBus<GPIOB_BASE, 12, 13>(st1);
        }
    }


    CanPublisher* getPublisher(uint16_t can_topic){
        return new CanPublisher(can_topic, can_manager->getBus(BUS_ID));
    }


    CanSocket* getSubscriber(uint16_t can_topic){
        CanSocket* socket = new CanSocket(can_topic);
        socket->open(can_manager->getBus(BUS_ID));
        return socket;
    }





};

#endif //CAN_ABSTRACTION_H