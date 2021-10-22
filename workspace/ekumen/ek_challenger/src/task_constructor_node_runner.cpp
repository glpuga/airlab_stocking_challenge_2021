/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// project
#include <ek_challenger/task_constructor_node.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior_tree_node_runner");
    ros::NodeHandle nh_{"~"};

    ek_challenger::TaskConstructorNode bt;
    return bt.run() ? 0 : -1;
}
