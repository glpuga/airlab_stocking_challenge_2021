/* Copyright (C) 2021 Gerardo Puga - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// stdlib
#include <map>
#include <cmath>

// gtest
#include <gtest/gtest.h>

// behaviortree.cpp
#include "behaviortree_cpp_v3/decorators/subtreemock_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

// the auxiliar tree nodes used in this test
#include "gtet_subtreemock_aux_nodes.h"

using namespace BT;

class SubtreeMockTests : public ::testing::Test
{
  public:
    const double tolerance_{1e-6};
    const std::string xml_text = R"(

<root main_tree_to_execute = "MainTree" >

    <BehaviorTree ID="MainTree">
        <Sequence>
            <SetBlackboard value="1.0" output_key="a" />
            <SetBlackboard value="4.0" output_key="b" />
            <SetBlackboard value="3.0" output_key="c" />
            <SubTree ID="QuadraticEquationSolver" a="a" b="b" c="c" res1="res1" res2="res2" />
        </Sequence>
    </BehaviorTree>

    <BehaviorTree ID="QuadraticEquationSolver">
        <Sequence>
            <SubTreePlus ID="SimpleSolutionSolver" a="{a}" b="{b}" c="{c}" sign="1.0" res="{res1}" />
            <SubTreePlus ID="SimpleSolutionSolver" a="{a}" b="{b}" c="{c}" sign="-1.0" res="{res2}" />
        </Sequence>
    </BehaviorTree>

    <BehaviorTree ID="SimpleSolutionSolver">
        <Sequence>
            <MultNode op1="2.0" op2="{a}" out="{denominator}" />
            <MultNode op1="-1.0" op2="{b}" out="{numerator_t1}" />
            <SubTree ID="DeterminantSolver" a="a" b="b" c="c" res="determinant" />
            <SqrtNode in="{determinant}" out="{abs_numerator_t2}" />
            <MultNode op1="{sign}" op2="{abs_numerator_t2}" out="{numerator_t2}" />
            <SumNode op1="{numerator_t1}" op2="{numerator_t2}" out="{numerator}" />
            <SumNode op1="{numerator_t1}" op2="{numerator_t2}" out="{numerator}" />
            <DivNode op1="{numerator}" op2="{denominator}" out="{res}" />
        </Sequence>
    </BehaviorTree>

    <BehaviorTree ID="DeterminantSolver">
        <Sequence>
            <MultNode op1="{b}" op2="{b}" out="{bb}" />
            <MultNode op1="{a}" op2="{c}" out="{ac}" />
            <MultNode op1="4.0" op2="{ac}" out="{ac4}" />
            <DiffNode op1="{bb}" op2="{ac4}" out="{res}" />
        </Sequence>
    </BehaviorTree>

</root> )";

    BehaviorTreeFactory factory_;

    std::map<std::string, SubtreeMockNode::CallbackFunction> mock_callbacks_;

    void SetUp() override
    {
        // fill in the factory with the nodes used by the tree
        factory_.registerNodeType<SumNode>("SumNode");
        factory_.registerNodeType<DiffNode>("DiffNode");
        factory_.registerNodeType<MultNode>("MultNode");
        factory_.registerNodeType<DivNode>("DivNode");
        factory_.registerNodeType<SqrtNode>("SqrtNode");

        // fill the calbacks map with the mock mock_callbacks_ that will replace branches in some of the tests
        mock_callbacks_["SimpleSolutionSolver"] = [](SubtreeMockBlackboardProxy& p) {
            static int state = 0;
            auto a = p.getInput<double>("a").value();
            auto b = p.getInput<double>("b").value();
            auto c = p.getInput<double>("c").value();

            switch (state++)
            {
                case 0:
                    p.setOutput<double>("res1", a + b + c);
                    break;
                case 1:
                    p.setOutput<double>("res2", a - b - c);
                    break;
                default:
                    break;
            }
            return NodeStatus::SUCCESS;
        };

        mock_callbacks_["QuadraticEquationSolver"] = [](SubtreeMockBlackboardProxy& p) {
            p.setOutput<double>("res1", 97.0);
            p.setOutput<double>("res2", 99.0);
            return NodeStatus::FAILURE;
        };
    }
};

TEST_F(SubtreeMockTests, FullRootTree)
{
    // Test the full tree, starting from the default root.
    // Read reasults from the blackboard and test final values.
    Blackboard::Ptr blackboard = Blackboard::create();
    Tree tree = factory_.createTreeFromText(xml_text.c_str(), blackboard);
    auto ret = tree.tickRoot();
    ASSERT_EQ(ret, NodeStatus::SUCCESS);

    auto sol_1_value = blackboard->get<double>("res1");
    auto sol_2_value = blackboard->get<double>("res2");
    ASSERT_NEAR(-1.0, sol_1_value, tolerance_);
    ASSERT_NEAR(-3.0, sol_2_value, tolerance_);
}

TEST_F(SubtreeMockTests, TestRootMockingSubtrees)
{
    // Test the full tree, mocking the values returned by the QuadraticEquationSolver subtree.
    // Read reasults from the blackboard and test final values.
    NodeBuilder mock_builder = [this](const std::string& name, const NodeConfiguration& config) {
        return std::make_unique<SubtreeMockNode>(name, config, mock_callbacks_.at(name));
    };

    factory_.registerBuilder<SubtreeMockNode>("SubtreeMock", mock_builder);

    Blackboard::Ptr blackboard = Blackboard::create();
    Tree tree = factory_.createTreeFromText(xml_text.c_str(), blackboard, "MainTree", true);
    auto ret = tree.tickRoot();

    ASSERT_EQ(ret, NodeStatus::FAILURE);
    auto sol_1_value = blackboard->get<double>("res1");
    auto sol_2_value = blackboard->get<double>("res2");
    ASSERT_NEAR(97.0, sol_1_value, tolerance_);
    ASSERT_NEAR(99.0, sol_2_value, tolerance_);
}

TEST_F(SubtreeMockTests, TestFullSubtree)
{
    // Test the QuadraticEquationSolver subtree, without mocking any subtrees.
    // Inputs get set on the bb before running the tree.
    // Inputs are such that the subtree returns SUCCEESS and provides results.
    Blackboard::Ptr blackboard = Blackboard::create();
    Tree tree =
        factory_.createTreeFromText(xml_text.c_str(), blackboard, "QuadraticEquationSolver");

    blackboard->set<double>("a", 1.0);
    blackboard->set<double>("b", -20.0);
    blackboard->set<double>("c", 99.0);
    auto ret = tree.tickRoot();

    ASSERT_EQ(ret, NodeStatus::SUCCESS);
    auto sol_1_value = blackboard->get<double>("res1");
    auto sol_2_value = blackboard->get<double>("res2");
    ASSERT_NEAR(11.0, sol_1_value, tolerance_);
    ASSERT_NEAR(9.0, sol_2_value, tolerance_);
}

TEST_F(SubtreeMockTests, TestSubtreeWithInputThatWouldCauseFAILURE)
{
    // Test QuadraticEquationSolver subtree, without mocking any subtrees.
    // Inputs get set on the bb before running the tree.
    // Inputs are such that the subtree returns FAILURE with no results in the blackboard.
    Blackboard::Ptr blackboard = Blackboard::create();
    Tree tree =
        factory_.createTreeFromText(xml_text.c_str(), blackboard, "QuadraticEquationSolver");

    // these inputs cause the determinant to be negative
    blackboard->set<double>("a", 2.0);
    blackboard->set<double>("b", 1.0);
    blackboard->set<double>("c", 2.0);
    auto ret = tree.tickRoot();

    ASSERT_EQ(ret, NodeStatus::FAILURE);
}

TEST_F(SubtreeMockTests, TestSubtreeMockingChildSubtrees)
{
    // Test QuadraticEquationSolver subtree, mocking subtrees called from there,
    NodeBuilder mock_builder = [this](const std::string& name, const NodeConfiguration& config) {
        return std::make_unique<SubtreeMockNode>(name, config, mock_callbacks_[name]);
    };

    factory_.registerBuilder<SubtreeMockNode>("SubtreeMock", mock_builder);

    Blackboard::Ptr blackboard = Blackboard::create();
    Tree tree =
        factory_.createTreeFromText(xml_text.c_str(), blackboard, "QuadraticEquationSolver", true);

    blackboard->set<double>("a", 0.0);
    blackboard->set<double>("b", 32.0);
    blackboard->set<double>("c", 10.0);
    auto ret = tree.tickRoot();

    ASSERT_EQ(ret, NodeStatus::SUCCESS);
    auto sol_1_value = blackboard->get<double>("res1");
    auto sol_2_value = blackboard->get<double>("res2");
    ASSERT_NEAR(42.0, sol_1_value, tolerance_);
    ASSERT_NEAR(-42.0, sol_2_value, tolerance_);
}

TEST_F(SubtreeMockTests, TestMockingWithNoSubtreeMockNodeThrows)
{
    // Tests that attempting to set mock_subtrees to true causes the creation of
    // the tree to throw if no SubtreeMock builder has been set in the factory.
    Blackboard::Ptr blackboard = Blackboard::create();
    ASSERT_THROW(
        {
            Tree tree = factory_.createTreeFromText(xml_text.c_str(), blackboard,
                                                    "QuadraticEquationSolver", true);
        },
        RuntimeError);
}

TEST_F(SubtreeMockTests, TestBadSubtreeIdThrows)
{
    Blackboard::Ptr blackboard = Blackboard::create();
    ASSERT_THROW(
        {
            Tree tree = factory_.createTreeFromText(xml_text.c_str(), blackboard,
                                                    "ThisSubtreeDoesNotExist");
        },
        RuntimeError);
}
