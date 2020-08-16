#include "RosInterface.h"

using namespace inet;

// register module class with OMNeT++
Define_Module(RosInterface);

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

        //////////////////////////////
        std_msgs::Int8MultiArray rosMsg;
        rosMsg.data.push_back(1);
        rosMsg.data.push_back(2);

        auto pkt = new Packet;
        auto data = makeShared<BytesChunk>();
        int d0 = rosMsg.data[0];
        int d1 = rosMsg.data[1];
        data->setBytes({d0, d1});
        pkt->insertAtBack(data);
        sendDirect(pkt, gates.at(0));
        //////////////////////////////

        // schedule next call
        scheduleAt(simTime() + 1, msg); //TOTO simTime
    }
}

void RosInterface::callback(const std_msgs::Int8MultiArrayConstPtr &msg, cGate *gate) {
    // modify pkt
    auto pkt = new Packet;
    auto data = makeShared<BytesChunk>();
    int d0 = msg->data[0];
    int d1 = msg->data[1];
    data->setBytes({d0, d1});
    pkt->insertAtBack(data);

    // send pkt to corresponding node
    sendDirect(pkt, gate);
}

void RosInterface::publish(cModule* module, cMessage *msg) {
    std::string nodeName = module->getOwner()->getName();

    // modify pkt
    std_msgs::Int8MultiArray rosMsg;

    //////////////////////////////
    auto pkt = check_and_cast<Packet *>(msg);
    auto data = pkt->popAtBack<BytesChunk>(B(2));
    int d0 = data->getByte(0);
    int d1 = data->getByte(1);
    rosMsg.data.push_back(d0);
    rosMsg.data.push_back(d1);
//    EV_INFO << data->str() << "\n";
    //////////////////////////////

    // send pkt to ros
    pubs.find(nodeName)->second.publish(rosMsg);
    delete msg;
}

void RosInterface::addRos(cModule* module) {
    cGate *gate = module->gate("rosIn");
    std::string nodeName = module->getOwner()->getName();
    std::string subTopic = "from_" + nodeName;
    std::string pubTopic = "to_" + nodeName;

    //////////////////////////////
    gates.push_back(gate);
    //////////////////////////////

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
