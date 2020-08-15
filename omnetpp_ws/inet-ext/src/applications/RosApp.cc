#ifndef __ROSAPP_H
#define __ROSAPP_H

#include "ros/RosInterface.h"
#include "inet/common/ModuleAccess.h"
#include "inet/applications/ethernet/EtherAppClient.h"
#include "inet/applications/ethernet/EtherApp_m.h"
#include "inet/linklayer/common/Ieee802Ctrl.h"
#include "inet/linklayer/common/Ieee802SapTag_m.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/can/RosMsg_m.h"
#include "inet/common/packet/Packet.h"

using namespace inet;

class RosApp : public ApplicationBase
{
  protected:
    RosInterface *ros = nullptr;

  protected:
    virtual void initialize(int stage) override;

    // ApplicationBase:
    virtual void handleMessageWhenUp(cMessage *msg) override;
    virtual void handleStartOperation(LifecycleOperation *operation) override;
    virtual void handleStopOperation(LifecycleOperation *operation) override;
    virtual void handleCrashOperation(LifecycleOperation *operation) override;
};

#endif // ifndef __ROSAPP_H

//#include "RosApp.h"
//
//using namespace inet;

// register module class with OMNeT++
Define_Module(RosApp);

/// TODO ///
/// message lowerLayer with IDENTIFIER

void RosApp::initialize(int stage) {
    ApplicationBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        ros = getModuleFromPar<RosInterface>(par("rosModule"), this);
        ros->addRos(this);
    }
}

void RosApp::handleMessageWhenUp(cMessage *msg) {
    if (msg->arrivedOn("rosIn")) {
        // send to bus
        send(msg, "lowerLayerOut");
    }
    else {
        // send to ros
        ros->publish(this, msg);
    }
}

void RosApp::handleStartOperation(LifecycleOperation *operation) {
    EV_INFO << "Starting application\n";
}

void RosApp::handleStopOperation(LifecycleOperation *operation) {
    EV_INFO << "Stop the application\n";
}

void RosApp::handleCrashOperation(LifecycleOperation *operation) {
    EV_INFO << "Crash the application\n";
}

