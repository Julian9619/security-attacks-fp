
#ifndef __ATTACKERAPP_H
#define __ATTACKERAPP_H

#include "inet/common/INETDefs.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/packet/chunk/BytesChunk.h"
//#include "inet/can/CanHeader_m.h"

#include <omnetpp/cmessage.h>
#include <omnetpp/cobjectfactory.h>
#include <omnetpp/csimulation.h>
#include <omnetpp/platdep/platdefs.h>
#include <omnetpp/regmacros.h>
#include <omnetpp/simtime.h>

#include "../../../../samples/inet/src/inet/applications/base/ApplicationBase.h"
#include "../../../../samples/inet/src/inet/can/RosMsg_m.h"
#include "../../../../samples/inet/src/inet/common/packet/chunk/FieldsChunk.h"
#include "../../../../samples/inet/src/inet/common/Ptr.h"
#include "../../../../samples/inet/src/inet/common/Units.h"

using namespace inet;

class AttackerApp : public ApplicationBase
{
  private:
    B canHeaderLength = B(8);

  protected:
    virtual void initialize() override {};
    virtual void handleMessage(cMessage *msg) override;

    // ApplicationBase:
        virtual void handleMessageWhenUp(cMessage *msg) override;
        virtual void handleStartOperation(LifecycleOperation *operation) override;
        virtual void handleStopOperation(LifecycleOperation *operation) override;
        virtual void handleCrashOperation(LifecycleOperation *operation) override;
};

#endif // ifndef __CANCONTROLLER_H

//#include "CanController.h"
//
//using namespace inet;

// register module class with OMNeT++
Define_Module(AttackerApp);

void AttackerApp::initialize() {
    // create selfMsg
    scheduleAt(simTime(), new cMessage("selfMessage"));
}

void AttackerApp::handleMessageWhenUp(cMessage *msg) {
    if (msg->isSelfMessage()) {

        auto pkt = new Packet;
        auto data = makeShared<RosMsg>();
        data->setChunkLength(rosMsgLength);
        data->setId(1);
        data->setData(7);
        pkt->insertAtBack(data);

        // send pkt to corresponding node
        sendDirect(pkt, gate);
        // schedule next call
        scheduleAt(simTime() + 1, msg); //TOTO simTime
    }
}

