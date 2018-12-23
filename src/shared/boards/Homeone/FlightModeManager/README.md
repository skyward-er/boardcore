 ```plantuml

skinparam titleFontSize 34

title Flight Mode Manager State Machine
top to bottom direction
state Init
state Testing
state Error
state Disarmed
state Armed
state Launching
state Ascending
state FirstDescentPhase
state SecondDescentPhase
state ManualDescent
state Landed

[*] --> Init
Init -u-> Error : EV_INIT_ERROR
Init --> Disarmed : EV_INIT_OK

Testing --> Testing : EV_TC_BOARD_RESET

Error --> Error : EV_TC_BOARD_RESET

Disarmed -u-> Error : EV_NC_OFFLINE
Disarmed -u-> Error : EV_IGN_OFFLINE
Disarmed -u-> Error : EV_NCEV_IGN_ABORTED_OFFLINE
Disarmed --> Armed : EV_TC_ARM
Disarmed --> Testing : EV_TC_TEST_MODE


Armed -u-> Error : EV_NC_OFFLINE
Armed -u-> Error : EV_IGN_OFFLINE
Armed -u-> Error : EV_NC_OFFLINE

Armed --> Disarmed : EV_TC_DISARM
Armed --> Disarmed : EV_TIMEOUT_ARM
Armed --> Launching : EV_TC_LAUNCH

Launching --> Ascending : EV_UMBILICAL_DETACHED
Launching --> Disarmed : EV_TC_DISARM
Launching -u-> Error : EV_IGN_ABORTED

Ascending --> FirstDescentPhase : EV_ADA_APOGEE_DETECTED
Ascending --> FirstDescentPhase : EV_TIMEOUT_APOGEE

FirstDescentPhase --> SecondDescentPhase : EV_ADA_DPL_ALT_DETECTED
FirstDescentPhase --> SecondDescentPhase : EV_TIMEOUT_DPL_ALT
FirstDescentPhase --> ManualDescent : EV_TC_MANUAL_MODE

SecondDescentPhase --> Landed : EV_TC_END_MISSION
SecondDescentPhase --> Landed : EV_TIMEOUT_END_MISSION

ManualDescent --> Landed : EV_TC_END_MISSION
ManualDescent --> Landed : EV_TIMEOUT_END_MISSION
 ```