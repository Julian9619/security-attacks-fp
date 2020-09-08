#ifndef __ROSINTERFACE_H
#define __ROSINTERFACE_H

#include "ros/ros.h"
#include "std_msgs/Int8MultiArray.h"
#include "inet/common/INETDefs.h"
#include "inet/common/packet/Packet.h"
#include "inet/applications/base/ApplicationBase.h"

using namespace inet;

class RosInterface : public cSimpleModule
{
  private:
    simtime_t delay = -1;
    std::vector<cGate *> gates;
    std::vector<ros::Subscriber> subs;
    std::map<std::string, ros::Publisher> pubs;

  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;

    virtual void callback(const std_msgs::Int8MultiArrayConstPtr &msg, cGate *gate);

  public:
    void publish(cModule* module, cMessage *msg);
    void addRos(cModule* module);
};

#endif // ifndef __ROSINTERFACE_H
