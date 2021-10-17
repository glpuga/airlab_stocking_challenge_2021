/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// standard library
#include <string>

// project
#include <ek_challenger/bt_challenger.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior_tree_node_runner");
    ros::NodeHandle nh_{"~"};

    ek_challenger::BehaviorTreeNode bt;
    return bt.run() ? 0 : -1;
}
