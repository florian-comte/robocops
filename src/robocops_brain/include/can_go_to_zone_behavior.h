#pragma once

#include "behaviortree_cpp/behavior_tree.h"

class CanGoToZone : public BT::SyncActionNode
{
public:
  CanGoToZone(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};
