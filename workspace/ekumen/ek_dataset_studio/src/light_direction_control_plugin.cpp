// standard library
#include <memory>

// gazebo
#include <gazebo/msgs/light.pb.h>
#include <gazebo/common/Console.hh>

// ros
#include <ros/ros.h>

// project
#include <ek_dataset_studio/light_direction_control_plugin.hpp>

namespace gazebo
{

  LightDirectionControlPlugin::LightDirectionControlPlugin()
  {
  }

  LightDirectionControlPlugin::~LightDirectionControlPlugin()
  {
    if (ros_nh_ptr_)
    {
      ros_nh_ptr_->shutdown();
    }
  }

  void LightDirectionControlPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    // initialized gazebo
    gz_nh_ptr_.reset(new gazebo::transport::Node());
    gz_nh_ptr_->Init();
    gz_pub_ptr_ = gz_nh_ptr_->Advertise<msgs::Light>(gz_light_topic_name_);

    // create the ros service
    ros_nh_ptr_ = std::make_unique<ros::NodeHandle>("~");
    ros_svr_ptr_ = std::make_unique<ros::ServiceServer>(ros_nh_ptr_->advertiseService(ros_service_name_, &LightDirectionControlPlugin::serviceCallback, this));
  }

  bool LightDirectionControlPlugin::serviceCallback(ek_dataset_studio::LightDirectionControl::Request &req,
                                                    ek_dataset_studio::LightDirectionControl::Response &rep)
  {
    msgs::Light msg;

    msg.set_name("sun");
    msg.set_type(msgs::Light::DIRECTIONAL);

    msg.mutable_pose()->mutable_position()->set_x(0);
    msg.mutable_pose()->mutable_position()->set_y(0);
    msg.mutable_pose()->mutable_position()->set_z(0);

    msg.mutable_pose()->mutable_orientation()->set_x(req.orientation.x);
    msg.mutable_pose()->mutable_orientation()->set_y(req.orientation.y);
    msg.mutable_pose()->mutable_orientation()->set_z(req.orientation.z);
    msg.mutable_pose()->mutable_orientation()->set_w(req.orientation.w);

    gz_pub_ptr_->Publish(msg);
    return true;
  }
}