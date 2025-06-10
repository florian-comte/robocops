#include "can_go_to_zone_behavior.h"
#include <iostream>

CanGoToZone::CanGoToZone(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config)
{
}

BT::PortsList CanGoToZone::providedPorts()
{
    return {
        BT::InputPort<int>("zone_index"),
        BT::InputPort<int>("MAX_INVENTORY"),
        BT::InputPort<int>("current_inventory"),
        BT::InputPort<std::vector<int>>("MIN_TO_GO_ZONES"),
        BT::InputPort<std::vector<int>>("TOTAL_ZONES"),
        BT::InputPort<std::vector<int>>("current_grabbed_zones"),
        BT::InputPort<std::vector<int>>("never_timed_out_zones")};
}

BT::NodeStatus CanGoToZone::tick()
{
    int zone;
    if (!getInput("zone_index", zone))
    {
        throw BT::RuntimeError("Missing required input [zone_index]");
    }

    // std::cout << "Zone index: " << zone << std::endl;

    // Get inputs from blackboard
    int max_inventory, current_inventory;
    if (!getInput("MAX_INVENTORY", max_inventory) ||
        !getInput("current_inventory", current_inventory))
    {
        throw BT::RuntimeError("Missing inventory values");
    }

    // std::cout << "Max inventory: " << max_inventory << ", Current inventory: " << current_inventory << std::endl;

    // Get MIN_TO_GO_ZONES
    std::vector<int> min_to_go;
    if (!getInput("MIN_TO_GO_ZONES", min_to_go))
    {
        throw BT::RuntimeError("Missing MIN_TO_GO_ZONES");
    }

    // std::cout << "MIN_TO_GO_ZONES: ";
    // for (const auto &val : min_to_go)
    // {
    //     std::cout << val << " ";
    // }
    // std::cout << std::endl;

    // Get TOTAL_ZONES
    std::vector<int> total_zones;
    if (!getInput("TOTAL_ZONES", total_zones))
    {
        throw BT::RuntimeError("Missing TOTAL_ZONES");
    }

    // std::cout << "TOTAL_ZONES: ";
    // for (const auto &val : total_zones)
    // {
    //     std::cout << val << " ";
    // }
    // std::cout << std::endl;

    // Get current_grabbed_zones
    std::vector<int> current_grabbed;
    if (!getInput("current_grabbed_zones", current_grabbed))
    {
        throw BT::RuntimeError("Missing current_grabbed_zones");
    }

    // std::cout << "current_grabbed_zones: ";
    // for (const auto &val : current_grabbed)
    // {
    //     std::cout << val << " ";
    // }
    // std::cout << std::endl;

    std::vector<bool> never_timed_out_zones;
    bool never_timed_out_door;
    if (!getInput("never_timed_out_zones", never_timed_out_zones))
    {
        throw BT::RuntimeError("Missing timeout flags");
    }

    // std::cout << "never_timed_out_zones: ";
    // for (const auto &val : never_timed_out_zones)
    // {
    //     std::cout << (val ? "true" : "false") << " ";
    // }
    // std::cout << ", never_timed_out_opening_door: " << (never_timed_out_door ? "true" : "false") << std::endl;

    if (zone >= static_cast<int>(min_to_go.size()) ||
        zone >= static_cast<int>(total_zones.size()) ||
        zone >= static_cast<int>(current_grabbed.size()) ||
        zone >= static_cast<int>(never_timed_out_zones.size()))
    {
        throw BT::RuntimeError("Zone index out of bounds");
    }

    // std::cout << "Zone index is within bounds" << std::endl;

    bool allowed =
        (max_inventory - current_inventory > min_to_go[zone]) &&
        (total_zones[zone] - current_grabbed[zone] > min_to_go[zone]) &&
        never_timed_out_zones[zone];

    // std::cout << "Evaluating allowed condition:" << std::endl;
    // std::cout << "  max_inventory - current_inventory > min_to_go[zone]: " << (max_inventory - current_inventory > min_to_go[zone]) << std::endl;
    // std::cout << "  total_zones[zone] - current_grabbed[zone] > min_to_go[zone]: " << (total_zones[zone] - current_grabbed[zone] > min_to_go[zone]) << std::endl;
    // std::cout << "  never_timed_out_zones[zone]: " << (never_timed_out_zones[zone] ? "true" : "false") << std::endl;
    // std::cout << "  never_timed_out_door || zone != 2: " << (never_timed_out_door || zone != 2) << std::endl;

    return allowed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
