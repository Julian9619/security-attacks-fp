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
    class CollisionModel : public ModelPlugin
    {
        public:CollisionModel() : ModelPlugin(){

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
            std::bind(&CollisionModel::OnUpdate, this));

            //initialisiere ROS
            if (!ros::isInitialized())
            {
                int argc = 0;
                char** argv = NULL;
                ros::init(argc, argv, "dein_Client_Name_TODO", ros::init_options::NoSigintHandler |
                                        ros::init_options::AnonymousName);
            }

            rosNode.reset(new ros::NodeHandle("dein_Client_Name_TODO"));
            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Int8MultiArray>(
                "/to_motor", 1,
                boost::bind(&CollisionModel::OnRosMsg, this, _1), 
                ros::VoidPtr(),&rosQueue);

            rosSub = rosNode->subscribe(so);
            rosQueueThread =std::thread(std::bind(&CollisionModel::QueueThread, this));

            sender_pub = n.advertise<std_msgs::Int8MultiArray>("from_motor", 1000);

            itCounter = 1000;

            collisionDetected = false;
            
            //Topic für OMNET++ der Nachricht uebergeben (hier z.B. 1)
            //zwei mal push_back um die data Eintraege zu erzeugen. Ansonsten entstehet ein Speicher-Fehler
            
            ///////////TODO////////////TODO/////////////
            own_msg.data.push_back(2); //CAN-Topic
            own_msg.data.push_back(0); //initalisialer Wert für Nachricht (in diesem fall -1 == request)
            ///////////TODO////////////TODO/////////////

            this->model->GetJoint("left_wheel_hinge")->SetVelocity(0, -2.0);
            this->model->GetJoint("right_wheel_hinge")->SetVelocity(0, -2.0);
            ROS_WARN("setVelocity test");


        }//end of load

        void OnUpdate()
        {          

            //sende RequestNachricht für SensorCollision alle 10 iterationen
            if(itCounter==6000){
                std_msgs::Int8MultiArray requestmsg;
                requestmsg.data.push_back(1); //CAN-Topic
                requestmsg.data.push_back(-1); //initalisialer Wert für Nachricht (in diesem fall -1 == request)
                if(ros::ok()) {
                    ROS_WARN("models sends: %d %d", requestmsg.data[0], requestmsg.data[1]);
                    sender_pub.publish(requestmsg);
                }
                itCounter = 0;
                ROS_WARN("%f",this->model->GetJoint("left_wheel_hinge")->GetVelocity(0));
                ROS_WARN("%f",this->model->GetJoint("right_wheel_hinge")->GetVelocity(0));
                if(collisionDetected) {
                    this->model->GetJoint("left_wheel_hinge")->SetVelocity(0, 0.0);
                    this->model->GetJoint("right_wheel_hinge")->SetVelocity(0, 0.0);
                }
                else {
                    this->model->GetJoint("left_wheel_hinge")->SetVelocity(0, -2.0);
                    this->model->GetJoint("right_wheel_hinge")->SetVelocity(0, -2.0);
                }
            }
            itCounter++;


        }//end of update

        
        public: void OnRosMsg(const std_msgs::Int8MultiArrayConstPtr &msg) 
        {
            ROS_WARN("model recieved: %d %d", msg->data[0], msg->data[1]);
            if(msg->data[0]==1){
                if(msg->data[1]==1){
                    collisionDetected = true;
                    ROS_WARN("%d", collisionDetected);
                }
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
    GZ_REGISTER_MODEL_PLUGIN(CollisionModel)
}
