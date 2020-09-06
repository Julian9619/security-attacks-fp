#include <omnetpp/cmessage.h>
#include <omnetpp/cobjectfactory.h>
#include <omnetpp/csimplemodule.h>
#include <omnetpp/csimulation.h>
#include <omnetpp/platdep/platdefs.h>
#include <omnetpp/regmacros.h>
#include <omnetpp/simtime.h>

#include "inet/common/packet/chunk/BytesChunk.h"
#include "inet/common/packet/Packet.h"
#include "../MsgType.cc"

#ifndef __CANBUS_H
#define __CANBUS_H

using namespace inet;

class CanBus : public cSimpleModule
{
  private:
    cMessage *currentMsg = nullptr;
    Packet *freeSignal = nullptr;

    simtime_t delay = -1;
    simtime_t interFrameSpace = -1;

    cMessage *busData = nullptr;
    cMessage *busFree = nullptr;

  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;

    virtual void merge(cMessage *msg);
    virtual void sendUp(cMessage *msg);
};

#endif // ifndef __CANBUS_H

//#include "CanBus.h"
//
//using namespace inet;

// register module class with OMNeT++
Define_Module(CanBus);

void CanBus::initialize() {
    freeSignal = new Packet;
    freeSignal->setKind(BUSFREE);
    auto dataField = makeShared<BytesChunk>();
    dataField->setBytes({0});
    freeSignal->insertAtBack(dataField);

    delay = par("delay");
    interFrameSpace = par("interFrameSpace");

    busData = new cMessage("BusData");
    busFree = new cMessage("BusFree");
}

void CanBus::handleMessage(cMessage *msg) {
    if(msg->isSelfMessage()) {
        if(msg == busData) {
            sendUp(currentMsg);
            delete currentMsg;
            currentMsg = nullptr;
            cancelEvent(busFree);
            scheduleAt(simTime()+interFrameSpace, busFree);
        } else if(msg == busFree) {
            sendUp(freeSignal);
        }
    } else {
        if(currentMsg == nullptr) {
            currentMsg = msg;
            scheduleAt(simTime()+delay, busData);
        } else {
            merge(msg);
        }
    }
}

void CanBus::merge(cMessage *msg) {
    Packet *frame = dynamic_cast<Packet *>(msg);
    Packet *currentFrame = dynamic_cast<Packet *>(currentMsg);
    int frameSize = frame->getBitLength();
    //Pop data
    auto data = frame->peekDataAsBits();
    auto currentData = currentFrame->popAtFront<BitsChunk>(b(frameSize));
    //modify it
    auto tmpData = (Ptr<BitsChunk>) currentData->dup();
    for(int i=0; i<frameSize; i++) {
        if( !data->getBit(i) ) tmpData->setBit(i, 0);
    }
    delete frame;
    //Push data back
    currentFrame->insertAtBack(tmpData);
}

void CanBus::sendUp(cMessage *msg) {
    int size = gateSize("cang$o");
    for (int i = 0; i < size; i++) {
        send(msg->dup(), "cang$o", i);
    }
}

