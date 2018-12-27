PinObserver pinObs();
MotorDriver motor(&pinObs);

CanManager canMgr(CAN1);
NoseconeManager noseconeMgr();

void noseconeInit()
{
    sEventBroker->start();
    initCanbus(canMgr);
    pinObs.start();
    noseconeMgr->start();
}