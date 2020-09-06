#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <gazebo/physics/physics.hh>
#include <thread>
#include <unistd.h>

namespace gazebo
{
    class MotorPlugin : public ModelPlugin
    {
        public:MotorPlugin() : ModelPlugin(){

        }

        /// \brief Pointer to the model
        private: physics::ModelPtr model;

        /// \brief Connection that maintains a link between the contact model's
        /// updated signal and the OnUpdate callback.
        private: event::ConnectionPtr updateConnection;

        private: bool collisionDetected;

        private: std::unique_ptr<ros::NodeHandle> rosNode;

        private: ros::NodeHandle n;

        private: ros::Publisher sender_pub;

        private: ros::Subscriber rosSub;
        
        private: ros::CallbackQueue rosQueue;
        
        private: std::thread rosQueueThread;
        private: int itCounter;

        private: int customer;

        private: int targetCoordinate;

        private: int order;

        private: std_msgs::Int8MultiArray own_msg;
        /* 
        Dies ist die Nachricht, die an OMNET++ gesendet wird. Beachte:
        msg.data[0] = 'ID (die ID entspricht dem Topic mit dem ein CAN-Node Nachrichten pubished)'
        msg.data[1...] = 'Daten die über CAN versendet werden sollen (nur Int erlaubt!!!!)'
        */



        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            this->model = _model;

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&MotorPlugin::OnUpdate, this));

            //initialisiere ROS
            if (!ros::isInitialized())
            {
                int argc = 0;
                char** argv = NULL;
                ros::init(argc, argv, "Car", ros::init_options::NoSigintHandler |
                                        ros::init_options::AnonymousName);
            }

            rosNode.reset(new ros::NodeHandle("Car"));
            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Int8MultiArray>(
                "/to_motor", 1,
                boost::bind(&MotorPlugin::OnRosMsg, this, _1), 
                ros::VoidPtr(),&rosQueue);

            rosSub = rosNode->subscribe(so);
            rosQueueThread =std::thread(std::bind(&MotorPlugin::QueueThread, this));

            sender_pub = n.advertise<std_msgs::Int8MultiArray>("from_motor", 1000);

            itCounter = 1000;

            customer = 2;

            order = 1;

            targetCoordinate = -1;  

            ROS_WARN("PLugin loaded");        

        }//end of load

        void start() {
            this->model->GetJoint("left_wheel_hinge")->SetVelocity(0, -3.0);
            this->model->GetJoint("right_wheel_hinge")->SetVelocity(0, -3.0);
        }

        void stop() {
            this->model->GetJoint("left_wheel_hinge")->SetVelocity(0, 0);
            this->model->GetJoint("right_wheel_hinge")->SetVelocity(0, 0);
        }

        void makeOrder(int num) {
            
        }

        int getX() {
            int x,y,z;
            gazebo::math::Pose pose;     
            pose = this->model->GetWorldPose();
            math::Vector3 v(0, 0, 0);
            v = pose.pos;
            x = v.x; // x coordinate
            y = v.y; // y coordinate
            z = v.z; // z coordinate
            //ROS_WARN("car: %d, %d, %d", x, y, z);
            return x;
        }

        void OnUpdate()
        {   
            if(this->getX()>=targetCoordinate) {
                this->stop();
            }
            else{
                start();
            }

/*
            if (this->customer == 0) {
                this->stop();
            }
            else if (this->customer == 1) {
                if (this->getX() >= 3) {
                    this->stop();
                }
                else {
                    this->start();
                }
            }
            else if (this->customer == 2) {
                if (this->getX() >= 7) {
                    this->stop();
                }
                else {
                    this->start();
                }
            }
*/
        }//end of update

        
        public: void OnRosMsg(const std_msgs::Int8MultiArrayConstPtr &msg) 
        {
            ROS_WARN("motor received: %d", msg->data[1]);

            if(msg->data[0]==1){
                targetCoordinate = msg->data[1];
            }
        }

        
        
        // ROS helper function that processes messages
        private: void QueueThread() 
        {
            static const double timeout = 0.1;
            while (rosNode->ok()) 
            {
                rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        
    };
    GZ_REGISTER_MODEL_PLUGIN(MotorPlugin)
}