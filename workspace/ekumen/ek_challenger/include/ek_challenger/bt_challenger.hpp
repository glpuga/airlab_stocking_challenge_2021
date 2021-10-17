/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>
#include <memory>

// third party
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <ros/ros.h>

namespace ek_challenger
{

    class BehaviorTreeNode
    {
    public:
        BehaviorTreeNode();

        bool run();

    private:
        const int32_t tick_rate_{10};

        ros::NodeHandle nh_;

        std::unique_ptr<BT::Tree> tree_;
        std::unique_ptr<BT::StdCoutLogger> cout_logger_;
        std::unique_ptr<BT::PublisherZMQ> zmq_logger_;

        void registerNodes(BT::BehaviorTreeFactory &factory);
    };

}
