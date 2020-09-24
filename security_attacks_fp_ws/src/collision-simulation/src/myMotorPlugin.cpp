
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
     * This Plugin is controlling the vehicles speed and reacts to incoming messages
     **/
    class MotorPlugin : public ModelPlugin
    {
        // brief constructor
        public:MotorPlugin() : ModelPlugin(){

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

        // the coordinate the vehicle is driving to
        private: int targetCoordinate;

        /** the load function is mandatory and links this plugin to a specific SDF element. 
         * Furthermore it sets some of the classes attributes to specific values
         **/
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            //store teh pointer to the model
            this->model = _model;

            //create a updateConnection to lsiten to call the OnUpdate function on evers simulation iteration
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&MotorPlugin::OnUpdate, this));

            //initialize ROS
            if (!ros::isInitialized())
            {
                int argc = 0;
                char** argv = NULL;
                ros::init(argc, argv, "Car", ros::init_options::NoSigintHandler |
                                        ros::init_options::AnonymousName);
            }

            //create the ROS-node
            rosNode.reset(new ros::NodeHandle("Car"));

            /** subscribe to a topic
             * our naming convention was to show to which plugin messages are sent. In this case
             * messages are sent from omnet through ROS "to motor", so the topic is "/to_motor"
             * On each incoming message the OnRosMsg function will be called
             * **/
            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Int8MultiArray>(
                "/to_motor", 1,
                boost::bind(&MotorPlugin::OnRosMsg, this, _1), 
                ros::VoidPtr(),&rosQueue);
            
            //create the subscriber
            rosSub = rosNode->subscribe(so);

            //spin up the queue helper thread
            rosQueueThread =std::thread(std::bind(&MotorPlugin::QueueThread, this));

            //create the publisher
            sender_pub = n.advertise<std_msgs::Int8MultiArray>("from_motor", 1000);

            //initialize the targetCoordinate
            targetCoordinate = 0;  

            ROS_WARN("PLugin loaded");        
        }

        //function to start the vehicle moving forward
        void start() {
            this->model->GetJoint("left_wheel_hinge")->SetVelocity(0, -3.0);
            this->model->GetJoint("right_wheel_hinge")->SetVelocity(0, -3.0);
        }

        // function to stop the vehicle
        void stop() {
            this->model->GetJoint("left_wheel_hinge")->SetVelocity(0, 0);
            this->model->GetJoint("right_wheel_hinge")->SetVelocity(0, 0);
        }

        /** function to return the vehicles current position
         * because the vehicle just drvies on the x-axis we are only
         * interessted in the x value
         * **/
        int getX() {
            int x,y,z;
            gazebo::math::Pose pose;     
            pose = this->model->GetWorldPose();
            math::Vector3 v(0, 0, 0);
            v = pose.pos;
            x = v.x; // x coordinate
            y = v.y; // y coordinate
            z = v.z; // z coordinate
            return x;
        }

        /** this method is called on every simulation iteration.
         * In this case it dose only check if it has reached its targetposition and stops,
         * otherwise the method will keep the vehicle running
         * **/
        void OnUpdate()
        {   
            if(this->getX()>=targetCoordinate) {
                this->stop();
            }
            else{
                start();
            }
        }

        /** this method is called to handle incoming messages
         * In this case it recieves a new targetposition and
         * updates its own
         * **/
        public: void OnRosMsg(const std_msgs::Int8MultiArrayConstPtr &msg) 
        {
            ROS_WARN("motor received: %d", msg->data[0]);
            targetCoordinate = msg->data[0];
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
    GZ_REGISTER_MODEL_PLUGIN(MotorPlugin)
}