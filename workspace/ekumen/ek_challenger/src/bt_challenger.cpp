/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard library
#include <string>

// third party
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/console.h>

// project
#include <ek_challenger/bt_challenger.hpp>
#include <ek_challenger/nodes/pause_action_node.hpp>
#include <ek_challenger/nodes/move_base_action_node.hpp>

namespace ek_challenger
{

    BehaviorTreeNode::BehaviorTreeNode() : nh_{"~"}
    {
        BT::BehaviorTreeFactory factory;
        registerNodes(factory);

        std::string tree_description_filepath;
        if (!ros::param::get("~tree_description_filepath", tree_description_filepath))
        {
            throw std::runtime_error("The behavior tree filepath parameter is missing");
        }

        ROS_WARN_STREAM("Creating behavior tree from description file: " << tree_description_filepath);
        tree_ = std::make_unique<BT::Tree>(factory.createTreeFromFile(tree_description_filepath));

        ROS_WARN_STREAM("Attaching CoutLogger to tree");
        cout_logger_ = std::make_unique<BT::StdCoutLogger>(*tree_);
        ROS_WARN_STREAM("Attaching ZMQ Publisher to tree");
        zmq_logger_ = std::make_unique<BT::PublisherZMQ>(*tree_);
    }

    bool BehaviorTreeNode::run()
    {
        ros::Rate rate(tick_rate_);
        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
            tree_->tickRoot();
        }
        return true;
    }

    void BehaviorTreeNode::registerNodes(BT::BehaviorTreeFactory &factory)
    {
        factory.registerNodeType<PauseActionNode>("Pause");
        factory.registerNodeType<MoveBaseActionNode>("MoveBase");
    }

}
