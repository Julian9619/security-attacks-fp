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
    class postbox2Plugin : public ModelPlugin
    {
        public:postbox2Plugin() : ModelPlugin(){

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
            std::bind(&postbox2Plugin::OnUpdate, this));

            //initialisiere ROS
            if (!ros::isInitialized())
            {
                int argc = 0;
                char** argv = NULL;
                ros::init(argc, argv, "Postbox2", ros::init_options::NoSigintHandler |
                                        ros::init_options::AnonymousName);
            }

            rosNode.reset(new ros::NodeHandle("Postbox2"));
            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Int8MultiArray>(
                "/postbox", 1,
                boost::bind(&postbox2Plugin::OnRosMsg, this, _1), 
                ros::VoidPtr(),&rosQueue);

            rosSub = rosNode->subscribe(so);
            rosQueueThread =std::thread(std::bind(&postbox2Plugin::QueueThread, this));

            sender_pub = n.advertise<std_msgs::Int8MultiArray>("postbox", 1000);

            itCounter = 0;

            collisionDetected = false;
            
            //Topic für OMNET++ der Nachricht uebergeben (hier z.B. 1)
            //zwei mal push_back um die data Eintraege zu erzeugen. Ansonsten entstehet ein Speicher-Fehler
            
            ///////////TODO////////////TODO/////////////
            own_msg.data.push_back(2); //CAN-Topic
            own_msg.data.push_back(0); //initalisialer Wert für Nachricht (in diesem fall -1 == request)
            ///////////TODO////////////TODO/////////////

            //ROS_WARN("loaded postbox2");

        }//end of load

        void OnUpdate()
        {          

            //sende RequestNachricht für SensorCollision alle 10 iterationen
            if(itCounter==5000){
                std_msgs::Int8MultiArray requestmsg;
                requestmsg.data.push_back(7); //CAN-Topic
                if(ros::ok()) {
                    sender_pub.publish(requestmsg);
                    //ROS_WARN("postbox2 send");
                }
            }
            itCounter++;

        }//end of update

        
        public: void OnRosMsg(const std_msgs::Int8MultiArrayConstPtr &msg) 
        {
            if(msg->data[0]==1){
                if(msg->data[1]==1){
                    collisionDetected = true;
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
    GZ_REGISTER_MODEL_PLUGIN(postbox2Plugin)
}