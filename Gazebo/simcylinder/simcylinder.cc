#include <functional>
#include <thread>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

using namespace std;

namespace gazebo
{
  class CylinderPlugin : public ModelPlugin
  {

    private: physics::ModelPtr model;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    private: ros::Publisher pub_pos;
    //private: ros::Publisher pub_moving;
    private: ros::Publisher pub_object_sight;
    private: ros::Subscriber rosSub;
    private: ros::Subscriber endAction;
    private: ros::CallbackQueue rosQueue;
    private: ros::CallbackQueue rosQueueAction;
    private: std::thread rosQueueThread;
    private: std::thread rosQueueThreadAction;
    geometry_msgs::Pose pose_ee;
    std_msgs::Bool is_moving;
    std_msgs::Float64 peak_object; 
    int first_appearance;
    bool count_move;
    bool end_action;
    bool activate_placement;
    double old_x;
    double old_y;
    double old_z;
    float pos;

    public: CylinderPlugin() : ModelPlugin()
            {
                pose_ee.position.x = 0;
                pose_ee.position.y = 0;
                pose_ee.position.z = 0;
                pose_ee.orientation.x = 1.0;
                pose_ee.orientation.y = 0;
                pose_ee.orientation.z = 0;
                pose_ee.orientation.w = 0;
                old_x = 0;
                old_y = 0;
                old_z = 0;
                peak_object.data = -10.0;
                first_appearance = 0;
                count_move = false;
                is_moving.data = false;
                activate_placement = false;
            }

    public: void SendPosition()
            {

            }

    public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
            {
                pos = _msg->data;
                if(pos == 0)
                {
                        this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
                }
                if(pos == 1)
                {
                        this->model->SetLinearVel(ignition::math::Vector3d(0.3, 0, 0));
                }
                if(pos == 2)
                {
                        this->model->SetLinearVel(ignition::math::Vector3d(0, 0.3, 0));
                }
                if(pos == 3)
                {
                        this->model->SetLinearVel(ignition::math::Vector3d(-0.3, 0, 0));
                }
                if(pos == 4)
                {
                        this->model->SetLinearVel(ignition::math::Vector3d(0, -0.3, 0));
                }
                if(pos == 5)
                {
                        activate_placement = true;
                        std::cout<<"activate placement \n";
                }
                if(pos == 6)
                {
                        activate_placement = false;
                        std::cout<<"deactivate placement \n";
                }
                
            }
    public: void OnRosMsgAction(const std_msgs::BoolConstPtr &_msg)
            {
                end_action = _msg->data;
            }

    private: void QueueThread()
            {
                static const double timeout = 0.01;
                while (this->rosNode->ok())
                {
                        this->rosQueue.callAvailable(ros::WallDuration(timeout));
                }
            }

    private: void QueueThreadAction()
            {
                static const double timeout = 0.01;
                while (this->rosNode->ok())
                {
                        this->rosQueueAction.callAvailable(ros::WallDuration(timeout));
                }
            }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
            {
               // Store the pointer to the model
               this->model = _parent;

               // Listen to the update event. This event is broadcast every
               // simulation iteration.
               this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                  std::bind(&CylinderPlugin::OnUpdate, this));
                  if (!ros::isInitialized())
                  {
                     int argc = 0;
                     char **argv = NULL;
                     ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
                  }
                  this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
                  this->pub_pos = this->rosNode->advertise<geometry_msgs::Pose>("/cylinder_position", 30);
                  //this->pub_moving = this->rosNode->advertise<std_msgs::Bool>("/cylinder_motion", 30);
                  this->pub_object_sight = this->rosNode->advertise<std_msgs::Float64>("/cylinder_visible", 30);
                  ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>("/simcylinder/commands_object",1,boost::bind(&CylinderPlugin::OnRosMsg, this, _1),ros::VoidPtr(), &this->rosQueue);
                  this->rosSub = this->rosNode->subscribe(so);
                  this->rosQueueThread = std::thread(std::bind(&CylinderPlugin::QueueThread, this));
                  ros::SubscribeOptions sub_o = ros::SubscribeOptions::create<std_msgs::Bool>("/motion_panda/end_action",1,boost::bind(&CylinderPlugin::OnRosMsgAction, this, _1),ros::VoidPtr(), &this->rosQueueAction);
                  this->endAction = this->rosNode->subscribe(sub_o);
                  this->rosQueueThreadAction = std::thread(std::bind(&CylinderPlugin::QueueThreadAction, this));
                  activate_placement = false;

            }
            // Called by the world update start event
    public: void OnUpdate()
             {
               // Apply a small linear velocity to the model.
               double x,y,z;
               double diff_x,diff_y;
               //ignition::math::Pose3d pose;
               auto pose = this->model->WorldPose();
               ignition::math::Vector3d current_pose(0, 0, 0);
               current_pose = pose.Pos();
               x = current_pose.X(); // x coordinate
               y = current_pose.Y(); // y coordinate
               z = current_pose.Z(); // z coordinate
               if((x < 0.25) || (x > 0.6) || (y < -0.25) || (y > 0.25) || (z < 0.8))
                {
                        if(end_action == true && activate_placement == true)
                        {
                                ignition::math::Pose3d pose = this->model->WorldPose();
                                ignition::math::Vector3d pos = pose.Pos();
                                ignition::math::Quaterniond current_rot(0, 0, 0, 1);
                                pos = ignition::math::Vector3d(0.45,-0.15,1.1);
                                ignition::math::Pose3d home_pose(pos, current_rot);
                                this->model->SetWorldPose(home_pose);
                                this->model->ResetPhysicsStates();
                                this->model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
                                this->model->SetGravityMode(true);
                        }
                }
               pose_ee.position.x = x;
               pose_ee.position.y = y;
               pose_ee.position.z = z;
               this->pub_pos.publish(pose_ee);
               /**diff_x = std::abs(old_x - x);
               diff_y = std::abs(old_y - y);

               //((x < old_x+0.01) && (x > old_x-0.01)) && ((y < old_y+0.01) && (y > old_y-0.01)) && ((z < old_z+0.01) && (z > old_z-0.01))
               if(diff_x > 0.000001 || diff_y > 0.000001)
               {
                       is_moving.data = true;
               }
               else
               {
                       is_moving.data = false;
               }
               if(count_move != is_moving.data)
               {
                       first_appearance++;
               }
               if(first_appearance > 2)
               {
                       peak_object.data = 60.0;
               }
                this->pub_moving.publish(is_moving);**/
                if((x > 0.15) && (x < 0.7) && (y > -0.35) && (y < 0.35))
                {
                        peak_object.data = 55.0;
                }
                else
                {
                        peak_object.data = -10.0;
                }
                this->pub_object_sight.publish(peak_object);
                old_x = x;
                old_y = y;
                old_z = z;
                count_move = is_moving.data;
             }
             // Pointer to the model

  };
  GZ_REGISTER_MODEL_PLUGIN(CylinderPlugin)
}
