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
    /**
     * every gazebo model-plugin is implemented as a C++ class which inherets from ModelPlugin,
     * with specitifc mehtods which are explaint below
     * This Plugin is is recieving data by an abstract wireless communication and sends it to the models motor
     **/
    class RecieverPlugin : public ModelPlugin
    {
        // brief constructor
        public:RecieverPlugin() : ModelPlugin(){

        }

        // brief Pointer to the model
        private: physics::ModelPtr model;

        // brief Connection that maintains a link between the contact model's
        // updated signal and the OnUpdate callback.
        private: event::ConnectionPtr updateConnection;

        //a ros-node to handle the multithreaded subscriber
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        // a simple ros-node to handle the publisher
        private: ros::NodeHandle n;

        // the ros-publisher
        private: ros::Publisher sender_pub;

        // the ros-subscriber
        private: ros::Subscriber rosSub;
        
        //callbackqueue handles incoming messages
        private: ros::CallbackQueue rosQueue;
        
        // a thread keeps running the rosQueue
        private: std::thread rosQueueThread;


        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            //store teh pointer to the model
            this->model = _model;

            //create a updateConnection to lsiten to call the OnUpdate function on evers simulation iteration
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&RecieverPlugin::OnUpdate, this));

            //initialize ROS
            if (!ros::isInitialized())
            {
                int argc = 0;
                char** argv = NULL;
                ros::init(argc, argv, "dein_Client_Name_TODO", ros::init_options::NoSigintHandler |
                                        ros::init_options::AnonymousName);
            }

            //create the ROS-node
            rosNode.reset(new ros::NodeHandle("dein_Client_Name_TODO"));

            /** subscribe to a topic
             * Becouse we used ROS to simulate or wireless comunication, the reciever subscribes to messages 
             * with the "postbox"-topic. On each incoming message the OnRosMsg function will be called 
             * **/
            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Int8MultiArray>(
                "/postbox", 1,
                boost::bind(&RecieverPlugin::OnRosMsg, this, _1), 
                ros::VoidPtr(),&rosQueue);

            //create the subscriber
            rosSub = rosNode->subscribe(so);

            //spin up the queue helper thread
            rosQueueThread =std::thread(std::bind(&RecieverPlugin::QueueThread, this));

            //create the publisher
            sender_pub = n.advertise<std_msgs::Int8MultiArray>("from_reciever", 1000);
            
            ROS_WARN("Reciever loaded");
        }

        /**
         * this method is called on each simulation iteration. To make the plugins more generic we have created
         * such a mehtod in this plugin without using it
         * */
        void OnUpdate()
        {          

        }

        /**
         * this method is called if a message with the " posbox"-topic is recieved. In this case the reciever will 
         * send the recieved data to the motor 
         * */
        public: void OnRosMsg(const std_msgs::Int8MultiArrayConstPtr &msg) 
        {
            //create the message. We are using Int8MultiArrays for more genericity. It is comparable to vectors
            std_msgs::Int8MultiArray recieverMsg;

            //push_back to avoid segmentation faults
            recieverMsg.data.push_back(0);

            //write recieved data in new message
            recieverMsg.data[0] = msg->data[0];

            //publish message
            sender_pub.publish(recieverMsg);
            ROS_WARN("reveiver sended message: %d", recieverMsg.data[0]);
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

    //register the plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(RecieverPlugin)
}