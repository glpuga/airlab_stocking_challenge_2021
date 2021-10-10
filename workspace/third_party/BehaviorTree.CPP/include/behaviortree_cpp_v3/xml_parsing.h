#ifndef XML_PARSING_BT_H
#define XML_PARSING_BT_H

#include "behaviortree_cpp_v3/bt_parser.h"

namespace BT
{

/**
 * @brief The XMLParser is a class used to read the model
 * of a BehaviorTree from file or text and instantiate the
 * corresponding tree using the BehaviorTreeFactory.
 */
class XMLParser: public Parser
{
  public:
    XMLParser(const BehaviorTreeFactory& factory);

    ~XMLParser();

    XMLParser(const XMLParser& other) = delete;
    XMLParser& operator=(const XMLParser& other) = delete;

    void loadFromFile(const std::string& filename) override;

    void loadFromText(const std::string& xml_text) override;

   /**
    * @brief Instantiates a BehaviorTreee from the source, using a given blackboard and given root subtree.
    * @param[in] root_blackboard Root blackboard, used by the root of the tree.
    * @param[in] root_subtree_id Id of the subtree used as the root to build the tree from. If left empty
    *                            the default root discovery algorithm to be used.
    * @param[in] mock_subtrees   Replaces subtree instantiations with a special node that can be configured
    *                            to call mock callbacks for testing.
    */
    Tree instantiateTree(const Blackboard::Ptr& root_blackboard,
                         const std::string& root_subtree,
                         const bool mock_subtrees) override;

  private:

    struct Pimpl;
    Pimpl* _p;

};

void VerifyXML(const std::string& xml_text,
               const std::set<std::string> &registered_nodes);

std::string writeTreeNodesModelXML(const BehaviorTreeFactory& factory);

}

#endif   // XML_PARSING_BT_H
