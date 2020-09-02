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
    class CarModel : public ModelPlugin
    {
        public:CarModel() : ModelPlugin(){

        }

        /// \brief Pointer to the model
        private: physics::ModelPtr model;

        /// \brief Connection that maintains a link between the contact model's
        /// updated signal and the OnUpdate callback.
        private: event::ConnectionPtr updateConnection;

        private: std::unique_ptr<ros::NodeHandle> rosNode;

        private: ros::NodeHandle n;

        private: ros::Publisher sender_pub;

        private: ros::Subscriber rosSub;
        
        private: ros::CallbackQueue rosQueue;
        
        private: std::thread rosQueueThread;
        private: int itCounter;

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
            std::bind(&CarModel::OnUpdate, this));

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
                "/to_car", 1,
                boost::bind(&CarModel::OnRosMsg, this, _1), 
                ros::VoidPtr(),&rosQueue);

            rosSub = rosNode->subscribe(so);
            rosQueueThread =std::thread(std::bind(&CarModel::QueueThread, this));

            sender_pub = n.advertise<std_msgs::Int8MultiArray>("from_car", 1000);

            itCounter = 1000;
            
            //Topic für OMNET++ der Nachricht uebergeben (hier z.B. 1)
            //zwei mal push_back um die data Eintraege zu erzeugen. Ansonsten entstehet ein Speicher-Fehler

            ROS_WARN("car: %d, %d, %d", 1, 2, 3);
            
            ///////////TODO////////////TODO/////////////
            own_msg.data.push_back(2); //CAN-Topic
            own_msg.data.push_back(0); //initalisialer Wert für Nachricht (in diesem fall -1 == request)
            ///////////TODO////////////TODO/////////////
        }//end of load

        public: void OnRosMsg(const std_msgs::Int8MultiArrayConstPtr &msg) 
        {
            //ROS_WARN("car received: %d %d", msg->data[0], msg->data[1]);
            int num = 0;
            if(msg->data[0]==1){
                if(msg->data[1]==1){
                    
                }
            }

            if (true) {
                //this->start();
            }
            else {
                //this->stop();
            }

            if(getX() == 10) {
                //this->stop();
            }
        }

        void start() {
            const auto &jointController = this->model->GetJointController();
            jointController->SetVelocityTarget(this->model->GetJoint("right_wheel_hinge")->GetScopedName(), 2);
            jointController->SetVelocityTarget(this->model->GetJoint("left_wheel_hinge")->GetScopedName(), 2);
            jointController->SetVelocityTarget(this->model->GetJoint("central_wheel_hinge")->GetScopedName(), 2);
        }


        void stop() {
            const auto &jointController = this->model->GetJointController();
            jointController->SetVelocityTarget(this->model->GetJoint("right_wheel_hinge")->GetScopedName(), 0);
            jointController->SetVelocityTarget(this->model->GetJoint("left_wheel_hinge")->GetScopedName(), 0);
            jointController->SetVelocityTarget(this->model->GetJoint("central_wheel_hinge")->GetScopedName(), 0);
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
            return -x;
        }

        void OnUpdate()
        {          

            //sende RequestNachricht für SensorCollision alle x iterationen
            if(itCounter==1000){
                std_msgs::Int8MultiArray requestmsg;
                requestmsg.data.push_back(1); //CAN-Topic
                requestmsg.data.push_back(-1); //initalisialer Wert für Nachricht (in diesem fall -1 == request)
                if(ros::ok()) {
                    //ROS_WARN("car sends: %d %d", requestmsg.data[0], requestmsg.data[1]);
                    sender_pub.publish(requestmsg);
                }
            }
            itCounter++;
        }//end of update
        
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
    GZ_REGISTER_MODEL_PLUGIN(CarModel)
}