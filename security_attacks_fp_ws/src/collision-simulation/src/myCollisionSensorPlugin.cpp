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

namespace gazebo
{
    class CollisionSensor : public SensorPlugin
    {
        public:CollisionSensor() : SensorPlugin(){

        }

        /// \brief Pointer to the contact sensor
        private: sensors::ContactSensorPtr parentSensor;

        /// \brief Connection that maintains a link between the contact sensor's
        /// updated signal and the OnUpdate callback.
        private: event::ConnectionPtr updateConnection;

        private: std::unique_ptr<ros::NodeHandle> rosNode;

        private: ros::NodeHandle n;

        private: ros::Publisher sender_pub;

        private: ros::Subscriber rosSub;
        
        private: ros::CallbackQueue rosQueue;
        
        private: std::thread rosQueueThread;

        private: std_msgs::Int8MultiArray own_msg;
        /* 
        Dies ist die Nachricht, die an OMNET++ gesendet wird. Beachte:
        msg.data[0] = 'ID (die ID entspricht dem Topic mit dem ein CAN-Node Nachrichten pubished)'
        msg.data[1...] = 'Daten die über CAN versendet werden sollen (nur Int erlaubt!!!!)'
        */



        void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
        {
            // Get the parent sensor.
            this->parentSensor =
             std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

            if (!this->parentSensor)
            {
                gzerr << "CollisionSensor requires a ContactSensor.\n";
                return;
            }

            this->updateConnection = this->parentSensor->ConnectUpdated(
              std::bind(&CollisionSensor::OnUpdate, this));

            this->parentSensor->SetActive(true);

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
                "/to_sensor", 1,
                boost::bind(&CollisionSensor::OnRosMsg, this, _1), 
                ros::VoidPtr(),&rosQueue);

            rosSub = rosNode->subscribe(so);
            rosQueueThread =std::thread(std::bind(&CollisionSensor::QueueThread, this));

            sender_pub = n.advertise<std_msgs::Int8MultiArray>("from_sensor", 1000);

            
            
            //Topic für OMNET++ der Nachricht uebergeben (hier z.B. 1)
            //zwei mal push_back um die data Eintraege zu erzeugen. Ansonsten entstehet ein Speicher-Fehler
            
            ///////////TODO////////////TODO/////////////
            own_msg.data.push_back(1); //CAN-Topic
            own_msg.data.push_back(0); //initalisialer Wert für Nachricht
            ///////////TODO////////////TODO///////////

            ROS_WARN("SensorPlugin loaded");


        }//end of load

        void OnUpdate()
        {
            msgs::Contacts contacts;
            contacts = this->parentSensor->Contacts();
            if(contacts.contact_size()>1){
                this->own_msg.data[1]=1;
            }
            //ROS_WARN("sensor says: %d %d", msg.data[0], msg.data[1]);
        }//end of update

        public: void OnRosMsg(const std_msgs::Int8MultiArrayConstPtr &msg) 
        {
            ROS_WARN("sensor recieved: %d %d", msg->data[0], msg->data[1]);
            ROS_WARN("msg->data[0]==1: %d", msg->data[0]==1);
            
            if(msg->data[0]==1){
                if(msg->data[1]==-1){
                    ROS_WARN("send back");
                    //sende Nachricht
                    if(ros::ok()) {
                        ROS_WARN("sensor sends: %d %d", own_msg.data[0], own_msg.data[1]);
                        sender_pub.publish(own_msg);

                    }
                }
            }
        }//end of OnRosMsg

        
        
        // ROS helper function that processes messages
        private: void QueueThread() 
        {
            static const double timeout = 0.1;
            while (rosNode->ok()) 
            {
                rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }//end of QueueThread*/

        
    };
    GZ_REGISTER_SENSOR_PLUGIN(CollisionSensor)
}