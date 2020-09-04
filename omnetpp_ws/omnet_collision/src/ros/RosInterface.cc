#include "RosInterface.h"

using namespace inet;

// register module class with OMNeT++
Define_Module(RosInterface);

void RosInterface::callback(const std_msgs::Int8MultiArrayConstPtr &msg, cGate *gate) {
    // modify msg
    auto pkt = new Packet;
    auto data = makeShared<RosMsg>();
    data->setChunkLength(rosMsgLength);
    data->setId(msg->data[0]);
    data->setData(msg->data[1]);
    pkt->insertAtBack(data);

    // send pkt to corresponding node
    sendDirect(pkt, gate);
}

void RosInterface::initialize() {
    // ros init
    int argc = 0; char* argv[] = {};
    ros::init(argc, argv, "omnetpp");

    // create selfMsg
    scheduleAt(simTime(), new cMessage("selfMessage"));
}

void RosInterface::handleMessage(cMessage *msg) {
    if (msg->isSelfMessage()) {
        // ros spin
        ros::spinOnce();

        // schedule next call
        scheduleAt(simTime() + 1, msg); //TOTO simTime
    }
}

void RosInterface::publish(cModule* module, cMessage *msg) {
    std::string nodeName = module->getOwner()->getName();

    // modify msg
    std_msgs::Int8MultiArray rosMsg;
    auto pkt = check_and_cast<Packet *>(msg);
    auto data = pkt->popAtBack<RosMsg>(rosMsgLength);
    rosMsg.data.push_back(data->getId());
    rosMsg.data.push_back(data->getData());

    // send pkt to ros
    pubs.find(nodeName)->second.publish(rosMsg);
    delete msg;
}

void RosInterface::addRos(cModule* module) {
    cGate *gate = module->gate("rosIn");
    std::string nodeName = module->getOwner()->getName();
    std::string subTopic = "from_" + nodeName;
    std::string pubTopic = "to_" + nodeName;

    // ros pub/sub
    ros::NodeHandle n;
    subs.push_back(
            n.subscribe<std_msgs::Int8MultiArray>(subTopic, 1000, boost::bind(&RosInterface::callback, this, _1, gate))
    );
    pubs.insert(std::pair<std::string, ros::Publisher>(
            nodeName,
            n.advertise<std_msgs::Int8MultiArray>(pubTopic, 1000)
    ));
}
