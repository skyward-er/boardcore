

#pragma once

#include <miosix.h>

#include "SX127XCommon.h"

namespace Boardcore
{

class EbyteFrontend : public SX127x::ISX127XFrontend
{
public:
    EbyteFrontend(miosix::GpioPin tx_enable, miosix::GpioPin rx_enable)
        : tx_enable(tx_enable), rx_enable(rx_enable)
    {
    }

    bool isOnPaBoost() override { return true; }
    int maxInPower() override { return 15; }

    void enableRx() override { rx_enable.high(); }
    void disableRx() override { rx_enable.low(); }
    void enableTx() override { tx_enable.high(); }
    void disableTx() override { tx_enable.low(); }

private:
    miosix::GpioPin tx_enable;
    miosix::GpioPin rx_enable;
};

class RA01Frontend : public SX127x::ISX127XFrontend
{
public:
    RA01Frontend() = default;

    bool isOnPaBoost() override { return true; }
    int maxInPower() override { return 17; }

    void enableRx() override {}
    void disableRx() override {}
    void enableTx() override {}
    void disableTx() override {}
};

class Skyward433Frontend : public SX127x::ISX127XFrontend
{
public:
    Skyward433Frontend() = default;

    bool isOnPaBoost() override { return false; }
    int maxInPower() override { return 15; }

    void enableRx() override {}
    void disableRx() override {}
    void enableTx() override {}
    void disableTx() override {}
};

}  // namespace Boardcore
