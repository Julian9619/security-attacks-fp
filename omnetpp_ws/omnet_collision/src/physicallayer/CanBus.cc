#ifndef __CANBUS_H
#define __CANBUS_H

#include "inet/common/INETDefs.h"
#include "inet/common/packet/Packet.h"

using namespace inet;

class CanBus : public cSimpleModule
{
  protected:
    virtual void initialize() override {};
    virtual void handleMessage(cMessage *msg) override;
};

#endif // ifndef __CANBUS_H

//#include "CanBus.h"
//
//using namespace inet;

// register module class with OMNeT++
Define_Module(CanBus);

void CanBus::handleMessage(cMessage *msg) {
    //send back to all nodes
    int size = gateSize("cang$o");
    for (int i = 0; i < size; i++) {
        send(msg->dup(), "cang$o", i);
    }
    delete msg;
}
