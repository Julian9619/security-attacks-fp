#ifndef __PRIVILEGEESCALATION_H
#define __PRIVILEGEESCALATION_H

#include "ros/RosInterface.h"
#include "inet/common/ModuleAccess.h"
#include "inet/applications/ethernet/EtherAppClient.h"
#include "inet/applications/ethernet/EtherApp_m.h"
#include "inet/linklayer/common/Ieee802Ctrl.h"
#include "inet/linklayer/common/Ieee802SapTag_m.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/common/packet/Packet.h"

using namespace inet;

class PrivilegeEscalation : public ApplicationBase
{
  private:
    simtime_t delay = -1;

  protected:
    virtual void initialize(int stage) override;

    // ApplicationBase:
    virtual void handleMessageWhenUp(cMessage *msg) override;
    virtual void handleStartOperation(LifecycleOperation *operation) override;
    virtual void handleStopOperation(LifecycleOperation *operation) override;
    virtual void handleCrashOperation(LifecycleOperation *operation) override;
};

#endif // ifndef __PRIVILEGEESCALATION_H

//#include "PrivilegeEscalation.h"
//
//using namespace inet;

// register module class with OMNeT++
Define_Module(PrivilegeEscalation);

/// TODO ///
/// message lowerLayer with IDENTIFIER

void PrivilegeEscalation::initialize(int stage) {
    ApplicationBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        delay = par("delay");
        scheduleAt(simTime(), new cMessage("selfMessage"));
    }
}

void PrivilegeEscalation::handleMessageWhenUp(cMessage *msg) {
    if (msg->isSelfMessage()) {
        // send to bus
        auto pkt = new Packet;
        auto dataField = makeShared<BytesChunk>();
        dataField->setBytes({0});
        pkt->insertAtBack(dataField);

        send(pkt, "lowerLayerOut");
        scheduleAt(simTime()+delay, msg);
    } else {
        delete msg;
    }
}

void PrivilegeEscalation::handleStartOperation(LifecycleOperation *operation) {
    EV_INFO << "Starting application\n";
}

void PrivilegeEscalation::handleStopOperation(LifecycleOperation *operation) {
    EV_INFO << "Stop the application\n";
}

void PrivilegeEscalation::handleCrashOperation(LifecycleOperation *operation) {
    EV_INFO << "Crash the application\n";
}

