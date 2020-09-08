#ifndef __NOAPP_H
#define __NOAPP_H

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

class NoApp : public cSimpleModule {
    virtual void initialize() override {};
    virtual void handleMessage(cMessage *msg) override;
};

#endif // ifndef __NOAPP_H

//#include "NoApp.h"
//
//using namespace inet;

// register module class with OMNeT++
Define_Module(NoApp);

void NoApp::handleMessage(cMessage *msg) {
    delete msg;
}
