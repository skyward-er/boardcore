 ```plantuml
skinparam titleFontSize 34

title Deployment State Machine

state Ready { 
    [*] --> CutterIdle

    state CutterIdle
    state CuttingDrogue
    state CuttingMain
    state C <<choice>>
}

[*] --> Ready
CutterIdle --> CuttingMain : EV_TC_CUT_MAIN
CutterIdle -r-> CuttingDrogue : EV_CUT_DROGUE

CuttingDrogue --> C : EV_TIMEOUT_CUTTING
C --> CuttingMain : [cut_main = true] 
C --> CutterIdle : [cut_main = false] 

CuttingMain --> CutterIdle : EV_TIMEOUT_CUTTING

 ```