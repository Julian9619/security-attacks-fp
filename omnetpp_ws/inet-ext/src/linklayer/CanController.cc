#include <omnetpp/cfsm.h>
#include <omnetpp/checkandcast.h>
#include <omnetpp/clog.h>
#include <omnetpp/cmessage.h>
#include <omnetpp/cobjectfactory.h>
#include <omnetpp/cownedobject.h>
#include <omnetpp/cpar.h>
#include <omnetpp/csimulation.h>
#include <omnetpp/platdep/platdefs.h>
#include <omnetpp/regmacros.h>
#include <iostream>

#include "../../../../../../omnetpp_ws/inet/src/inet/common/FSMA.h"
#include "../../../../../../omnetpp_ws/inet/src/inet/common/InitStages.h"
#include "../../../../../../omnetpp_ws/inet/src/inet/common/packet/chunk/BytesChunk.h"
#include "../../../../../../omnetpp_ws/inet/src/inet/common/packet/Packet.h"
#include "../../../../../../omnetpp_ws/inet/src/inet/common/Ptr.h"
#include "../../../../../../omnetpp_ws/inet/src/inet/linklayer/base/MacProtocolBase.h"
#include "../../../../../../omnetpp_ws/inet/src/inet/queueing/contract/IPacketQueue.h"
#include "../MsgType.cc"

#ifndef __CANCONTROLLER_H
#define __CANCONTROLLER_H

//#include "inet/can/CanHeader_m.h"

using namespace inet;

class CanController : public MacProtocolBase
{
  protected:
    int identifier;

    enum State {
        IDLE,
        ARBITRATION,
        TRANSMIT,
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

        txQueue = check_and_cast<queueing::IPacketQueue *>(getSubmodule("queue"));
        fsm.setName("CanController State Machine");

        arbitrationMsg = new Packet;
        arbitrationMsg->setKind(PRIO);
        auto data = makeShared<BytesChunk>();
        data->setBytes({identifier});
        arbitrationMsg->insertAtBack(data);;
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
            FSMA_Event_Transition(Idle-Backoff,
                                  isLowerMessage(msg) && frame->getKind()==PRIO,
                                  BACKOFF,
            );
//            FSMA_Event_Transition(Idle-Receive,
//                                  isLowerMessage(msg) && !isArbitrationMsg(frame),
//                                  RECEIVE,
//            );
        }
        FSMA_State(ARBITRATION)
        {
            FSMA_Enter(sendDown(arbitrationMsg->dup()));
            FSMA_Event_Transition(Arbitration-Transmit,
                                  isLowerMessage(msg) && arbitrationSuccess(frame),
                                  TRANSMIT,
                                  delete msg;
            );
            FSMA_Event_Transition(Arbitration-Backoff,
                                  isLowerMessage(msg) && !arbitrationSuccess(frame),
                                  BACKOFF,
                                  delete msg;
            );
        }
        FSMA_State(BACKOFF)
        {
            FSMA_Event_Transition(Backoff-Receive,
                                  isLowerMessage(msg) && frame->getKind()==DATA,
                                  RECEIVE,
            );
            FSMA_Event_Transition(Backoff-Idle,
                                  isLowerMessage(msg) && frame->getKind()==BUSFREE,
                                  IDLE,
            );
        }
        FSMA_State(TRANSMIT)
        {
            FSMA_Enter(scheduleAt(simTime(), transmit));
            FSMA_Event_Transition(Transmit-Backoff,
                                  msg == transmit,
                                  BACKOFF,
                                  sendDown(currentTxFrame->dup());
                                  deleteCurrentTxFrame();
            );
        }
        FSMA_State(RECEIVE)
        {
            FSMA_Enter(sendUp(msg));
            FSMA_No_Event_Transition(Receive-Backoff,
                                     true,
                                     BACKOFF,
            );
        }
    }
    if (fsm.getState() == IDLE) {
        if (currentTxFrame != nullptr)
            handleWithFsm(currentTxFrame);
        else if (!txQueue->isEmpty()) {
            popTxQueue();
            handleWithFsm(currentTxFrame);
        }
    }
}

void CanController::encapsulate(Packet *frame) {
}

void CanController::decapsulate(Packet *frame) {
}

bool CanController::arbitrationSuccess(Packet *frame){
    if( frame->getKind() == PRIO ) {
        auto data = frame->peekDataAsBytes();
        int id = data->getByte(0);
        return id == identifier;
    }
    return false;
}

