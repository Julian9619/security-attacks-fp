#ifndef __CANCONTROLLER_H
#define __CANCONTROLLER_H

//#include "inet/can/CanHeader_m.h"
#include "inet/common/FSMA.h"
#include "inet/common/INETDefs.h"
#include "inet/common/packet/Packet.h"
#include "inet/linklayer/base/MacProtocolBase.h"

using namespace inet;

class CanController : public MacProtocolBase
{
  protected:
    enum State {
        IDLE,
        DEFER,
        TRANSMIT,
        RECEIVE,
        BACKOFF,
    };

    cFSM fsm;

    /** Remaining backoff period in seconds */
    simtime_t backoffPeriod = -1;

    /** Number of frame retransmission attempts. */
    int retryCounter = -1;

    B canHeaderLength = B(8);

  protected:
    virtual void initialize(int stage) override;
    virtual void configureInterfaceEntry() override;

    virtual void handleSelfMessage(cMessage *msg) override;
    virtual void handleUpperPacket(Packet *packet) override;
    virtual void handleLowerPacket(Packet *packet) override;
    virtual void handleWithFsm(cMessage *msg);

    virtual void encapsulate(Packet *frame);
    virtual void decapsulate(Packet *frame);

    void sendDataFrame(Packet *frame);
    void finishCurrentTransmission();
    bool isMediumFree();
};

#endif // ifndef __CANCONTROLLER_H

//#include "CanController.h"
//
//using namespace inet;

// register module class with OMNeT++
Define_Module(CanController);

/// TODO ///
/// Can Msg Frames,
/// Packet erzeugen

void CanController::initialize(int stage) {
    MacProtocolBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        // initialize self messages
//        endBackoff = new cMessage("Backoff");
//        endData = new cMessage("Data");

        // set up internal queue
        txQueue = check_and_cast<queueing::IPacketQueue *>(getSubmodule("queue"));

        // state variables
        fsm.setName("CsmaCaMac State Machine");
        backoffPeriod = -1;
        retryCounter = 0;
    }
    else if (stage == INITSTAGE_LINK_LAYER) {
    }
}

void CanController::configureInterfaceEntry() {
}

void CanController::handleSelfMessage(cMessage *msg)
{
    EV << "received self message: " << msg << endl;
//    handleWithFsm(msg);
}

void CanController::handleUpperPacket(Packet *packet) {
    auto frame = check_and_cast<Packet *>(packet);
    encapsulate(frame);

    EV << "received frame from higher layer: " << frame << endl;
//    txQueue->pushPacket(frame);
//    if (fsm.getState() != IDLE)
//        EV << "deferring upper message transmission in " << fsm.getStateName() << " state\n";
//    else if (!txQueue->isEmpty()){
//        popTxQueue();
//        handleWithFsm(currentTxFrame);
//    }
    send(packet, "lowerLayerOut");
}

void CanController::handleLowerPacket(Packet *packet) {
    EV << "received message from lower layer: " << packet << endl;
//    handleWithFsm(packet);
    send(packet, "upperLayerOut");
}

void CanController::handleWithFsm(cMessage *msg)
{
//    Packet *frame = dynamic_cast<Packet *>(msg);
    FSMA_Switch(fsm)
    {
        FSMA_State(IDLE)
        {
            FSMA_Event_Transition(Idle-Transmit,
                                  msg->arrivedOn("upperLayerIn"),
                                  TRANSMIT,
                                  scheduleAt(simTime(), currentTxFrame);
            );
            FSMA_Event_Transition(Idle-Recieve,
                                  msg->arrivedOn("lowerLayerIn"),
                                  RECEIVE,
                                  scheduleAt(simTime(), currentTxFrame);
            );
        }
        FSMA_State(TRANSMIT)
        {
            FSMA_Event_Transition(Transmit-Idle,
                                  true,
                                  IDLE,
                                  send(msg, "lowerLayerOut");
            );
        }
        FSMA_State(RECEIVE)
        {
            FSMA_Event_Transition(Recieve-Idle,
                                  true,
                                  IDLE,
                                  send(msg, "upperLayerOut");
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

void CanController::encapsulate(Packet *frame)
{
}

void CanController::decapsulate(Packet *frame)
{
}

void CanController::sendDataFrame(Packet *frame)
{
    EV << "sending Data frame " << frame->getName() << endl;
    send(frame, "lowerLayerOut");
}

void CanController::finishCurrentTransmission()
{
}

bool CanController::isMediumFree()
{
    return true;
}

