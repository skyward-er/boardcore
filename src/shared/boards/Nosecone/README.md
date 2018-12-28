# Nosecone Board

## Features

- Canbus communication
- H-Bridge control via PWM
- Motor limit control (*finecorsa*)
- H-Bridge current sensing
- SD logging
- reset if communication with main board is lost

## Components

 ```plantuml
skinparam class {
    BackgroundColor<<Hardware>> White
    ArrowColor<<Hardware>> Black
    BorderColor<<Hardware>> Black
}

package NoseconeBoard <<node>>
{
class Canbus << (A,#FF0000) ActiveObject >>
class NoseconeManager << (A,#FF0000) ActiveObject >>
class MotorDriver
class CurrentSensor << (A,#FF0000) ActiveObject >>
class PinObserver << (A,#FF0000) ActiveObject >>
class LogProxy
class Logger
class EvenBroker << (A,#FF0000) ActiveObject >>
}

class HomeoneBoard <<Hardware>>
class HB1 <<Hardware>>
class HB2 <<Hardware>>
class SDCard <<Hardware>>

hide members
hide <<Hardware>> circle

HomeoneBoard -down-> Canbus: CAN_MSG

Canbus -down-> EvenBroker: "EV_OPEN\nEV_CLOSE\nEV_STOP"
EvenBroker -right-> NoseconeManager: "forward\nevents"
Canbus -right-> LogProxy: getStatus()
LogProxy -right-> Logger: log()
NoseconeManager -down-> MotorDriver: "start()\nstop()"
NoseconeManager .right.> LogProxy: log()
MotorDriver .right.> CurrentSensor: create
MotorDriver .left.> PinObserver: "add limit pins"
CurrentSensor .up.> LogProxy: log()
PinObserver -up-> EvenBroker: "EV_LIMIT"

HB1 .up.> CurrentSensor: IS
HB2 .up.> CurrentSensor: IS

MotorDriver -down-> HB1: INH
MotorDriver -down-> HB1: IN
MotorDriver -down-> HB2: INH
MotorDriver -down-> HB2: IN

HB1 -right- HB2

Logger -down-> SDCard
 ```

## Description

* **Canbus**: The shared Canbus driver provides a way to read and write canbus messages. In particular, the function that has to be executed on message reception is defined in CanImpl.cpp.

* **FSM**: There is a very simple state machine that receives events from various components via the EventBroker. This last component is needed so the FSM can post timeout events.

* **Motor**: The motor driver is composed by
    * a driver for the h-bridge
    * an ActiveObject that samples the current value and logs it
    * two functions, defined in MotorLimit.h, that are passed to the PinObserver and called on the falling edge of these pins

* **Logging**: the logging is done via the shared Logger, but before this a LogProxy intercepts every call to the logger and updates the value of an internal status struct, that can be then retrieved and sent on the canbus (see CanImpl.cpp).