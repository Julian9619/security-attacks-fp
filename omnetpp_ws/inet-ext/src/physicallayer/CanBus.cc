#ifndef __CANBUS_H
#define __CANBUS_H

#include "inet/common/INETDefs.h"
#include "inet/common/packet/Packet.h"

using namespace inet;

class CanBus : public cSimpleModule
{
  private:
    cMessage *currentMsg = nullptr;
    cMessage *selfMsg = nullptr;

  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;

    void deleteCurrentMsg();
};

#endif // ifndef __CANBUS_H

//#include "CanBus.h"
//
//using namespace inet;

// register module class with OMNeT++
Define_Module(CanBus);

void CanBus::initialize() {
    selfMsg = new cMessage("SelfMsg");
}

void CanBus::handleMessage(cMessage *msg) {
    if (msg->isSelfMessage()) {
        //send currentMsg to all nodes
        int size = gateSize("cang$o");
        for (int i = 0; i < size; i++) {
            send(currentMsg->dup(), "cang$o", i);
        }
        deleteCurrentMsg();
    } else {
        if(currentMsg == nullptr) {
            scheduleAt(simTime()+1, selfMsg);
        } else {
            deleteCurrentMsg();
        }
        currentMsg = msg;
    }
}

void CanBus::deleteCurrentMsg() {
    delete currentMsg;
    currentMsg = nullptr;
}
