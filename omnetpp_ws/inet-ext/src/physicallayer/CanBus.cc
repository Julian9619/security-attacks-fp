#include <omnetpp/cmessage.h>
#include <omnetpp/cobjectfactory.h>
#include <omnetpp/csimplemodule.h>
#include <omnetpp/csimulation.h>
#include <omnetpp/platdep/platdefs.h>
#include <omnetpp/regmacros.h>
#include <omnetpp/simtime.h>

#include "../../../../../../omnetpp_ws/inet/src/inet/common/packet/chunk/BytesChunk.h"
#include "../../../../../../omnetpp_ws/inet/src/inet/common/packet/Packet.h"
#include "../MsgType.cc"

#ifndef __CANBUS_H
#define __CANBUS_H

using namespace inet;

class CanBus : public cSimpleModule
{
  private:
    cMessage *currentMsg = nullptr;
    cMessage *selfMsg = nullptr;
    Packet *busFree = nullptr;

  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;

    virtual bool isHigherPrio(cMessage *msg);
};

#endif // ifndef __CANBUS_H

//#include "CanBus.h"
//
//using namespace inet;

// register module class with OMNeT++
Define_Module(CanBus);

void CanBus::initialize() {
    selfMsg = new cMessage("SelfMsg");
    busFree = new Packet;
    busFree->setKind(BUSFREE);
}

void CanBus::handleMessage(cMessage *msg) {
    if(msg->isSelfMessage()) {
        if(msg->getKind() == BUSFREE) {
            int size = gateSize("cang$o");
            for (int i = 0; i < size; i++) {
                send(busFree->dup(), "cang$o", i);
            }
        } else {
            //send currentMsg to all nodes
            int size = gateSize("cang$o");
            for (int i = 0; i < size; i++) {
                send(currentMsg->dup(), "cang$o", i);
            }
            if(currentMsg->getKind() == DATA) {
                scheduleAt(simTime()+5, busFree);
            }
            delete currentMsg;
            currentMsg = nullptr;
        }
    } else {
        if(currentMsg == nullptr) {
            currentMsg = msg;
            scheduleAt(simTime()+1, selfMsg);
        } else if(isHigherPrio(msg)) {
            delete currentMsg;
            currentMsg = msg;
        }
    }
}

bool CanBus::isHigherPrio(cMessage *msg) {
    Packet *frame = dynamic_cast<Packet *>(msg);
    Packet *currentFrame = dynamic_cast<Packet *>(currentMsg);
    if( frame->getKind() == PRIO ) {
        auto data = frame->peekDataAsBytes();
        int id = data->getByte(0);
        auto currentData = currentFrame->peekDataAsBytes();
        int currentId = currentData->getByte(0);
        return id > currentId;
    }
    return false;
}

