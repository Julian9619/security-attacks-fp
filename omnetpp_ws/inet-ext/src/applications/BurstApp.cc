#ifndef __BURSTAPP_H
#define __BURSTAPP_H

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

class BurstApp : public ApplicationBase
{
  private:
    simtime_t delay = -1;
    int counter = 0;

  protected:
    virtual void initialize(int stage) override;

    // ApplicationBase:
    virtual void handleMessageWhenUp(cMessage *msg) override;
    virtual void handleStartOperation(LifecycleOperation *operation) override;
    virtual void handleStopOperation(LifecycleOperation *operation) override;
    virtual void handleCrashOperation(LifecycleOperation *operation) override;
};

#endif // ifndef __BURSTAPP_H

//#include "BurstApp.h"
//
//using namespace inet;

// register module class with OMNeT++
Define_Module(BurstApp);

/// TODO ///
/// message lowerLayer with IDENTIFIER

void BurstApp::initialize(int stage) {
    ApplicationBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        delay = par("delay");
        scheduleAt(simTime(), new cMessage("selfMessage"));
    }
}

void BurstApp::handleMessageWhenUp(cMessage *msg) {
    if (msg->isSelfMessage()) {
        if(counter < 4) {
            // send to bus
            auto pkt = new Packet;
            auto dataField = makeShared<BytesChunk>();
            dataField->setBytes({42});
            pkt->insertAtBack(dataField);
            send(pkt, "lowerLayerOut");

            scheduleAt(simTime()+0.1*delay, msg);
            counter++;
        } else {
            scheduleAt(simTime()+delay, msg);
            counter = 0;
        }
    } else {
        delete msg;
    }
}

void BurstApp::handleStartOperation(LifecycleOperation *operation) {
    EV_INFO << "Starting application\n";
}

void BurstApp::handleStopOperation(LifecycleOperation *operation) {
    EV_INFO << "Stop the application\n";
}

void BurstApp::handleCrashOperation(LifecycleOperation *operation) {
    EV_INFO << "Crash the application\n";
}

