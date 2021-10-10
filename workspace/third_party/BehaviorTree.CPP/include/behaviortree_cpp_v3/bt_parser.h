#ifndef PARSING_BT_H
#define PARSING_BT_H

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/blackboard.h"

namespace BT
{
/**
 * @brief The BehaviorTreeParser is a class used to read the model
 * of a BehaviorTree from file or text and instantiate the
 * corresponding tree using the BehaviorTreeFactory.
 */
class Parser
{
  public:
    Parser() = default;

    virtual ~Parser() = default;

    Parser(const Parser& other) = delete;
    Parser& operator=(const Parser& other) = delete;

    virtual void loadFromFile(const std::string& filename) = 0;

    virtual void loadFromText(const std::string& xml_text) = 0;

  /**
   * @brief Instantiates a BehaviorTreee from the source, using a given blackboard and given root subtree.
   * @param[in] root_blackboard Root blackboard, used by the root of the tree.
   * @param[in] root_subtree_id Id of the subtree used as the root to build the tree from. Defaults to empty,
   *                            which causes the default root discovery algorithm to be used.
   * @param[in] mock_subtrees   Replaces subtree instantiations with a special node that can be configured
   *                            to call mock callbacks for testing.
   */
    virtual Tree instantiateTree(const Blackboard::Ptr &root_blackboard,
                                 const std::string &root_subtree = "",
                                 const bool mock_subtrees = false) = 0;
};

}

#endif   // PARSING_BT_H
