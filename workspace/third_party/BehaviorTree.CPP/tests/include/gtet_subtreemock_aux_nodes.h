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

#ifndef GTEST_SUBTREEMOCK_AUX_NODES_H
#define GTEST_SUBTREEMOCK_AUX_NODES_H

// stdlib
#include <cmath>

// behaviortree.cpp
#include "behaviortree_cpp_v3/bt_factory.h"

class SumNode : public BT::SyncActionNode
{
  public:
    SumNode(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    BT::NodeStatus tick() override
    {
        auto op1 = getInput<double>("op1");
        auto op2 = getInput<double>("op2");
        if (!op1 || !op2)
        {
            throw BT::RuntimeError("missing required input for ", __PRETTY_FUNCTION__);
        }
        setOutput("out", op1.value() + op2.value());
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("op1"), BT::InputPort<double>("op2"),
                BT::OutputPort<double>("out")};
    }
};

class DiffNode : public BT::SyncActionNode
{
  public:
    DiffNode(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    BT::NodeStatus tick() override
    {
        auto op1 = getInput<double>("op1");
        auto op2 = getInput<double>("op2");
        if (!op1 || !op2)
        {
            throw BT::RuntimeError("missing required input for ", __PRETTY_FUNCTION__);
        }
        setOutput("out", op1.value() - op2.value());
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("op1"), BT::InputPort<double>("op2"),
                BT::OutputPort<double>("out")};
    }
};

class MultNode : public BT::SyncActionNode
{
  public:
    MultNode(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    BT::NodeStatus tick() override
    {
        auto op1 = getInput<double>("op1");
        auto op2 = getInput<double>("op2");
        if (!op1 || !op2)
        {
            throw BT::RuntimeError("missing required input for ", __PRETTY_FUNCTION__);
        }
        setOutput("out", op1.value() * op2.value());
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("op1"), BT::InputPort<double>("op2"),
                BT::OutputPort<double>("out")};
    }
};

class DivNode : public BT::SyncActionNode
{
  public:
    DivNode(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    BT::NodeStatus tick() override
    {
        auto op1 = getInput<double>("op1");
        auto op2 = getInput<double>("op2");
        if (!op1 || !op2)
        {
            throw BT::RuntimeError("missing required input for ", __PRETTY_FUNCTION__);
        }
        setOutput("out", op1.value() / op2.value());
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("op1"), BT::InputPort<double>("op2"),
                BT::OutputPort<double>("out")};
    }
};

class SqrtNode : public BT::SyncActionNode
{
  public:
    SqrtNode(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    BT::NodeStatus tick() override
    {
        auto op = getInput<double>("in");
        if (!op)
        {
            throw BT::RuntimeError("missing required input for ", __PRETTY_FUNCTION__);
        }
        if (op.value() < 0.0)
        {
            return BT::NodeStatus::FAILURE;
        }
        setOutput("out", std::sqrt(op.value()));
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("in"), BT::OutputPort<double>("out")};
    }
};

#endif
