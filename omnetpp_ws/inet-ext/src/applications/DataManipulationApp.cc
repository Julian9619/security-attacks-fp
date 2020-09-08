#include <omnetpp/cgate.h>
#include <omnetpp/clog.h>
#include <omnetpp/cmessage.h>
#include <omnetpp/cobjectfactory.h>
#include <omnetpp/cpar.h>
#include <omnetpp/csimplemodule.h>
#include <omnetpp/platdep/platdefs.h>
#include <omnetpp/regmacros.h>
#include <omnetpp/simtime_t.h>
#include <iostream>

#include "../../../../../../omnetpp_ws/inet/src/inet/applications/base/ApplicationBase.h"
#include "../../../../../../omnetpp_ws/inet/src/inet/common/InitStages.h"

#ifndef __DATAMANIPULATIONAPP_H
#define __DATAMANIPULATIONAPP_H

#include "ros/RosInterface.h"
#include "inet/common/ModuleAccess.h"
#include "inet/applications/ethernet/EtherAppClient.h"
#include "inet/applications/ethernet/EtherApp_m.h"
#include "inet/linklayer/common/Ieee802Ctrl.h"
#include "inet/linklayer/common/Ieee802SapTag_m.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/common/packet/Packet.h"
#include "../MsgType.cc"

using namespace inet;

class DataManipulationApp : public ApplicationBase
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

#endif // ifndef __DATAMANIPULATIONAPP_H

//#include "DataManipulationApp.h"
//
//using namespace inet;

// register module class with OMNeT++
Define_Module(DataManipulationApp);

/// TODO ///
/// message lowerLayer with IDENTIFIER

void DataManipulationApp::initialize(int stage) {
    ApplicationBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        delay = par("delay");
    }
}

void DataManipulationApp::handleMessageWhenUp(cMessage *msg) {
    Packet *frame = dynamic_cast<Packet *>(msg);
    if (msg->arrivedOn("lowerLayerIn") && frame->getKind()==PRIO) {
        Packet *pkt = new Packet;
        pkt->setKind(DATA);
        auto data = makeShared<BitsChunk>();
        data->setBits({0,0,1,0,1,0,1,0});
        pkt->insertAtBack(data);
        send(pkt, "lowerLayerOut");
    }
}

void DataManipulationApp::handleStartOperation(LifecycleOperation *operation) {
    EV_INFO << "Starting application\n";
}

void DataManipulationApp::handleStopOperation(LifecycleOperation *operation) {
    EV_INFO << "Stop the application\n";
}

void DataManipulationApp::handleCrashOperation(LifecycleOperation *operation) {
    EV_INFO << "Crash the application\n";
}

