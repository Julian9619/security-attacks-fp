#include <omnetpp/cfsm.h>
#include <omnetpp/checkandcast.h>
#include <omnetpp/clog.h>
#include <omnetpp/cmessage.h>
#include <omnetpp/cobjectfactory.h>
#include <omnetpp/cownedobject.h>
#include <omnetpp/cpar.h>
#include <omnetpp/platdep/platdefs.h>
#include <omnetpp/regmacros.h>
#include <iostream>
#include <algorithm>
#include <iostream>
#include <vector>

#include "inet/common/FSMA.h"
#include "inet/common/InitStages.h"
#include "inet/common/packet/chunk/BitsChunk.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/Ptr.h"
#include "inet/linklayer/base/MacProtocolBase.h"
#include "inet/queueing/contract/IPacketQueue.h"
#include "../MsgType.cc"

#ifndef __CANCONTROLLER_H
#define __CANCONTROLLER_H

//#include "inet/can/CanHeader_m.h"

using namespace inet;

class CanController : public MacProtocolBase
{
  protected:
    int identifier;
    std::vector<int> subscriber;

    enum State {
        IDLE,
        ARBITRATION,
        TRANSMIT,
        TRANSCEIVE,
        RECEIVE,
        BACKOFF,
    };
    cFSM fsm;

    Packet *arbitrationMsg = nullptr;
    cMessage *transmit = nullptr;

  public:
    virtual ~CanController();

  protected:
    virtual void initialize(int stage) override;
    virtual void configureInterfaceEntry() override;

    virtual void handleSelfMessage(cMessage *msg) override;
    virtual void handleUpperPacket(Packet *packet) override;
    virtual void handleLowerPacket(Packet *packet) override;
    virtual void handleWithFsm(cMessage *msg);

    virtual void encapsulate(Packet *frame);
    virtual void decapsulate(Packet *frame);

    virtual bool arbitrationSuccess(Packet *frame);
    virtual bool isSubscribedToMsg(Packet *frame);
};

#endif // ifndef __CANCONTROLLER_H

//#include "CanController.h"
//
//using namespace inet;

// register module class with OMNeT++
Define_Module(CanController);

CanController::~CanController() {
    cancelAndDelete(arbitrationMsg);
    cancelAndDelete(transmit);
}

/// TODO ///
/// Can Msg Frames,
/// Packet erzeugen

void CanController::initialize(int stage) {
    MacProtocolBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        identifier = par("identifier");
        subscriber = cStringTokenizer(par("subscriber").stringValue()).asIntVector();

        //initialize Queue
        txQueue = check_and_cast<queueing::IPacketQueue *>(getSubmodule("queue"));
        //initialize FSM
        fsm.setName("CanController State Machine");

        //Msg to win arbitration
        arbitrationMsg = new Packet;
        arbitrationMsg->setKind(PRIO);
        auto data = makeShared<BitsChunk>();
        data->setBits({0,0,0,0,0,0,0,0});
        for(int i=0; i<=identifier; i++) {
            data->setBit(i, 1);
        }
        arbitrationMsg->insertAtBack(data);
        //msg to schedule Transmission
        transmit = new cMessage("Transmit");
    }
    else if (stage == INITSTAGE_LINK_LAYER) {
    }
}

void CanController::configureInterfaceEntry() {
}

void CanController::handleSelfMessage(cMessage *msg) {
    EV << "received self message: " << msg << endl;
    handleWithFsm(msg);
}

void CanController::handleUpperPacket(Packet *packet) {
    EV << "received frame from higher layer: " << packet << endl;
    packet->setKind(DATA);
    txQueue->pushPacket(packet);
    if (fsm.getState() != IDLE)
        EV << "deferring upper message transmission in " << fsm.getStateName() << " state\n";
    else if (!txQueue->isEmpty()){
        popTxQueue();
        handleWithFsm(currentTxFrame);
    }
}

void CanController::handleLowerPacket(Packet *packet) {
    EV << "received message from lower layer: " << packet << endl;
    handleWithFsm(packet);
}

bool test(Packet *frame) {
    int b = frame->getKind();
    return b==DATA;
}

void CanController::handleWithFsm(cMessage *msg) {
    Packet *frame = dynamic_cast<Packet *>(msg);
    FSMA_Switch(fsm)
    {
        FSMA_State(IDLE)
        {
            FSMA_Event_Transition(Idle-Arbitration,
                                  isUpperMessage(msg),
                                  ARBITRATION,
            );
            FSMA_Event_Transition(Arbitration-Backoff,
                                  isLowerMessage(msg) && frame->getKind()==DATA,
                                  BACKOFF,
            );
            FSMA_Event_Transition(Idle-Receive,
                                  isLowerMessage(msg) && frame->getKind()==PRIO && isSubscribedToMsg(frame),
                                  RECEIVE,
            );
            FSMA_Event_Transition(Idle-Backoff,
                                  isLowerMessage(msg) && frame->getKind()==PRIO && !isSubscribedToMsg(frame),
                                  BACKOFF,
            );
        }
        FSMA_State(ARBITRATION)
        {
            FSMA_Enter(sendDown(arbitrationMsg->dup()));
            FSMA_Event_Transition(Arbitration-Idle,
                                  isLowerMessage(msg) && frame->getKind()==BUSFREE,
                                  IDLE,
            );
            FSMA_Event_Transition(Arbitration-Backoff,
                                  isLowerMessage(msg) && frame->getKind()==DATA,
                                  BACKOFF,
            );
            FSMA_Event_Transition(Arbitration-Transceive,
                                  isLowerMessage(msg) && frame->getKind()==PRIO && arbitrationSuccess(frame) && isSubscribedToMsg(frame),
                                  TRANSCEIVE,
            );
            FSMA_Event_Transition(Arbitration-Transmit,
                                  isLowerMessage(msg) && frame->getKind()==PRIO && arbitrationSuccess(frame) && !isSubscribedToMsg(frame),
                                  TRANSMIT,
            );
            FSMA_Event_Transition(Arbitration-Receive,
                                  isLowerMessage(msg) && frame->getKind()==PRIO && !arbitrationSuccess(frame) && isSubscribedToMsg(frame),
                                  RECEIVE,
            );
            FSMA_Event_Transition(Arbitration-Backoff,
                                  isLowerMessage(msg) && frame->getKind()==PRIO && !arbitrationSuccess(frame) && !isSubscribedToMsg(frame),
                                  BACKOFF,
            );
        }
        FSMA_State(BACKOFF)
        {
            FSMA_Event_Transition(Backoff-Idle,
                                  isLowerMessage(msg) && frame->getKind()==BUSFREE,
                                  IDLE,
            );
        }
        FSMA_State(TRANSCEIVE)
        {
            FSMA_Enter(sendDown(currentTxFrame->dup()));
            FSMA_Event_Transition(Transceive-Backoff,
                                  isLowerMessage(msg) && frame->getKind()==DATA,
                                  BACKOFF,
                                  deleteCurrentTxFrame();
                                  sendUp(msg);
            );
        }
        FSMA_State(TRANSMIT)
        {
            FSMA_Enter(sendDown(currentTxFrame->dup()));
            FSMA_Event_Transition(Transmit-Backoff,
                                  isLowerMessage(msg) && frame->getKind()==DATA,
                                  BACKOFF,
                                  deleteCurrentTxFrame();
            );
        }
        FSMA_State(RECEIVE)
        {
            FSMA_Event_Transition(Receive-Backoff,
                                  isLowerMessage(msg) && frame->getKind()==DATA,
                                  BACKOFF,
                                  sendUp(msg)
            );
        }
    }
    if(fsm.getState() == IDLE) {
        if (currentTxFrame != nullptr)
            handleWithFsm(currentTxFrame);
        else if (!txQueue->isEmpty()) {
            popTxQueue();
            handleWithFsm(currentTxFrame);
        }
    }
    if (isLowerMessage(msg) && frame->getOwner()==this) delete frame;
}

void CanController::encapsulate(Packet *frame) {
}

void CanController::decapsulate(Packet *frame) {
}

bool CanController::isSubscribedToMsg(Packet *frame) {
    auto data = frame->peekDataAsBits();
    int c=0;
    for(int i=0; i<=7; i++) {
        if(data->getBit(i)) c++;
    }
    return std::count(subscriber.begin(), subscriber.end(), c-1);
}

bool CanController::arbitrationSuccess(Packet *frame){
    auto data = frame->peekDataAsBits();
    for(int i=0; i<=identifier; i++) {
        if(!data->getBit(i)) return false;
    }
    return true;
}

