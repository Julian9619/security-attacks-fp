#ifndef __CANTRANSCIEVER_H
#define __CANTRANSCIEVER_H

#include "inet/common/INETDefs.h"
#include "inet/common/packet/Packet.h"

using namespace inet;

class CanTransciever : public cSimpleModule
{
  protected:
    virtual void initialize() override {};
    virtual void handleMessage(cMessage *msg) override;
};

#endif // ifndef __CANTRANSCIEVER_H

//#include "CanTransciever.h"
//
//using namespace inet;

// register module class with OMNeT++
Define_Module(CanTransciever);

/// TODO ///
/// Queue for Packets,
/// Bitwise Transmission + Arbitration

void CanTransciever::handleMessage(cMessage *msg) {
    if (msg->arrivedOn("upperLayerIn")) {
        send(msg, "phys$o");
    } else {
        send(msg, "upperLayerOut");
    }
}
