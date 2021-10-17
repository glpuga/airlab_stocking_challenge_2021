// standard library
#include <string>
#include <memory>

// gazebo
#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"

// ros
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>

// project
#include <ek_dataset_studio/LightDirectionControl.h>

namespace gazebo
{
    class LightDirectionControlPlugin : public WorldPlugin
    {
    public:
        LightDirectionControlPlugin();

        virtual ~LightDirectionControlPlugin();

        void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) override;

    private:
        const std::string ros_service_name_{"set_sun_direction"};
        const std::string gz_light_topic_name_{"~/light/modify"};

        std::unique_ptr<ros::NodeHandle>
            ros_nh_ptr_;
        std::unique_ptr<ros::ServiceServer> ros_svr_ptr_;

        gazebo::transport::NodePtr gz_nh_ptr_;
        gazebo::transport::PublisherPtr gz_pub_ptr_;

        bool serviceCallback(ek_dataset_studio::LightDirectionControl::Request &req,
                             ek_dataset_studio::LightDirectionControl::Response &rep);
    };

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(LightDirectionControlPlugin)
}