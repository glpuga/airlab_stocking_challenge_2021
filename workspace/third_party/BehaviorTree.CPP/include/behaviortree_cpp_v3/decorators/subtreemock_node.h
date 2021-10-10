#ifndef ACTION_SUBTREEMOCK_NODE_H
#define ACTION_SUBTREEMOCK_NODE_H

#include "behaviortree_cpp_v3/action_node.h"

#include <functional>
#include <string>

namespace BT
{
/**
 * @brief SubtreeMockBlackboardProxy is a wrapper class that provided limited access to
 *        the blackboard to mocked nodes. The mock callback receives modifiable reference to
 *        a SubtreeMockBlackboardProxy instance that can be used to access the
 *        BT blackboard.
 */
class SubtreeMockBlackboardProxy
{
  public:
    explicit SubtreeMockBlackboardProxy(TreeNode* mock_node) : mock_node_{mock_node}
    {
    }

    // thou shall not propagate or store this object
    SubtreeMockBlackboardProxy(const SubtreeMockBlackboardProxy&) = delete;
    SubtreeMockBlackboardProxy(SubtreeMockBlackboardProxy&&) = delete;

    template <typename T>
    Result getInput(const std::string& key, T& destination) const
    {
        return mock_node_->getInput<T>(key, destination);
    }

    template <typename T>
    Optional<T> getInput(const std::string& key) const
    {
        return mock_node_->getInput<T>(key);
    }

    template <typename T>
    Result setOutput(const std::string& key, const T& value)
    {
        return mock_node_->setOutput<T>(key, value);
    }

  private:
    TreeNode* mock_node_;
};

/**
 * @brief The SubtreeMockNode is a node that replaces a whole subtree, allowing to replace its
 *        execution with the execution of a standin callback function. This allows mocking subtrees
 *        to simplify the testing of large behavior trees.
 */
class SubtreeMockNode : public SyncActionNode
{
  public:
    using CallbackFunction = std::function<BT::NodeStatus(SubtreeMockBlackboardProxy& proxy)>;

    SubtreeMockNode(const std::string& name, const NodeConfiguration& config,
                    const CallbackFunction& callback)
      : SyncActionNode{name, config}, callback_{callback}
    {
    }

  private:
    CallbackFunction callback_;

    BT::NodeStatus tick() override
    {
        SubtreeMockBlackboardProxy bp(this);
        return callback_(bp);
    }
};

}   // namespace BT

#endif   // ACTION_SUBTREEMOCK_NODE_H
