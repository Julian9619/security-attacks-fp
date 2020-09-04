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
    B arbitrationLength = B(1);
    simtime_t backoffPeriod = 10us;

    enum State {
        IDLE,
        ARBITRATION,
        TRANSMIT,
        RECEIVE,
        DEFER,
        BACKOFF,
    };

    cFSM fsm;

    cMessage *endBackoff = nullptr;
    cMessage *endData = nullptr;

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
};

#endif // ifndef __CANCONTROLLER_H

//#include "CanController.h"
//
//using namespace inet;

// register module class with OMNeT++
Define_Module(CanController);

CanController::~CanController() {
    cancelAndDelete(endBackoff);
    cancelAndDelete(endData);
}

/// TODO ///
/// Can Msg Frames,
/// Packet erzeugen

void CanController::initialize(int stage) {
    MacProtocolBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        txQueue = check_and_cast<queueing::IPacketQueue *>(getSubmodule("queue"));

        fsm.setName("CanController State Machine");

        endBackoff = new cMessage("Backoff");
        endData = new cMessage("Data");
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
    encapsulate(packet);

    EV << "received frame from higher layer: " << packet << endl;
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
        }
        FSMA_State(ARBITRATION)
        {
            FSMA_Event_Transition(Arbitration-Transmit,
                                  isUpperMessage(msg),
                                  TRANSMIT,
                                  scheduleAt(simTime(), currentTxFrame);
            );
            FSMA_Event_Transition(Arbitration-Backoff,
                                  isUpperMessage(msg),
                                  BACKOFF,
                                  scheduleAt(simTime(), currentTxFrame);
            );
        }
        FSMA_State(BACKOFF)
        {
            FSMA_Event_Transition(Backoff-Idle,
                                  msg->arrivedOn("upperLayerIn"),
                                  IDLE,
                                  scheduleAt(simTime(), currentTxFrame);
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
    auto arbitrationField = makeShared<BytesChunk>();
    arbitrationField->setBytes({10});
    frame->insertAtFront(arbitrationField);
}

void CanController::decapsulate(Packet *frame) {
    frame->popAtFront<BytesChunk>(B(1));
}

