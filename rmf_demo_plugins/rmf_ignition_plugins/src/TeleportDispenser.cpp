/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <vector>

#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/AxisAlignedBox.hh>

#include <ignition/math/AxisAlignedBox.hh>

#include <rclcpp/rclcpp.hpp>

#include <rmf_plugins_common/dispenser_common.hpp>

using namespace ignition::gazebo;
using namespace rmf_dispenser_common;

namespace rmf_ignition_plugins {

class IGNITION_GAZEBO_VISIBLE TeleportDispenserPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
public:

  TeleportDispenserPlugin();
  ~TeleportDispenserPlugin();
  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>&,
    EntityComponentManager& ecm, EventManager&) override;
  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override;

private:
  // Stores params representing state of Dispenser, and handles all message pub/sub
  std::unique_ptr<TeleportDispenserCommon> _dispenser_common;

  Entity _dispenser;
  Entity _item_en; // Item that dispenser may contain
  ignition::math::AxisAlignedBox _dispenser_vicinity_box;

  bool tried_fill_dispenser = false; // Set to true if fill_dispenser() has been called at least once

  rclcpp::Node::SharedPtr _ros_node;

  bool find_nearest_model(
    EntityComponentManager& ecm,
    const std::vector<Entity>& robot_model_entities,
    Entity& robot_entity) const;
  void place_on_entity(EntityComponentManager& ecm,
    const Entity& base, const Entity& to_move);
  bool dispense_on_nearest_robot(EntityComponentManager& ecm,
    const std::string& fleet_name);
  void fill_dispenser(EntityComponentManager& ecm);
  void create_dispenser_bounding_box(EntityComponentManager& ecm);
};

TeleportDispenserPlugin::TeleportDispenserPlugin()
: _dispenser_common(std::make_unique<TeleportDispenserCommon>())
{
}

TeleportDispenserPlugin::~TeleportDispenserPlugin()
{
  rclcpp::shutdown();
}

bool TeleportDispenserPlugin::find_nearest_model(
  EntityComponentManager& ecm,
  const std::vector<Entity>& robot_model_entities,
  Entity& robot_entity) const
{
  double nearest_dist = 1e6;
  bool found = false;
  const auto dispenser_pos =
    ecm.Component<components::Pose>(_dispenser)->Data().Pos();

  for (const auto& en : robot_model_entities)
  {
    std::string name = ecm.Component<components::Name>(en)->Data();
    if (name == _dispenser_common->guid)
      continue;

    const auto en_pos = ecm.Component<components::Pose>(en)->Data().Pos();
    const double dist = en_pos.Distance(dispenser_pos);
    if (dist < nearest_dist)
    {
      nearest_dist = dist;
      robot_entity = en;
      found = true;
    }
  }
  return found;
}

// Move entity `to_move` onto `base`
void TeleportDispenserPlugin::place_on_entity(EntityComponentManager& ecm,
  const Entity& base, const Entity& to_move)
{
  const auto base_aabb = ecm.Component<components::AxisAlignedBox>(base);
  const auto to_move_aabb = ecm.Component<components::AxisAlignedBox>(to_move);
  auto new_pose = ecm.Component<components::Pose>(base)->Data();
  if (!base_aabb || !to_move_aabb)
  {
    RCLCPP_WARN(
      _dispenser_common->ros_node->get_logger(),
      "Either base entity or item to be dispensed does not have an AxisAlignedBox component. \
      Dispensing item to approximate location.");
    new_pose += ignition::math::Pose3<double>(0, 0, 0.5, 0, 0, 0);
  }
  else
  {
    new_pose.SetZ(base_aabb->Data().Max().Z() +
      0.5*(to_move_aabb->Data().ZLength()));
  }

  auto cmd = ecm.Component<components::WorldPoseCmd>(to_move);
  if (!cmd)
  {
    ecm.CreateComponent(to_move,
      components::WorldPoseCmd(ignition::math::Pose3<double>()));
  }
  ecm.Component<components::WorldPoseCmd>(to_move)->Data() = new_pose;
}

bool TeleportDispenserPlugin::dispense_on_nearest_robot(
  EntityComponentManager& ecm, const std::string& fleet_name)
{
  if (!_dispenser_common->dispenser_filled)
    return false;

  const auto fleet_state_it = _dispenser_common->fleet_states.find(fleet_name);
  if (fleet_state_it == _dispenser_common->fleet_states.end())
  {
    RCLCPP_WARN(_dispenser_common->ros_node->get_logger(),
      "No such fleet: [%s]", fleet_name.c_str());
    return false;
  }
  std::vector<Entity> robot_model_list;
  for (const auto& rs : fleet_state_it->second->robots)
  {
    std::vector<Entity> entities =
      ecm.EntitiesByComponents(components::Name(rs.name),
        components::Model(), components::Static(false));
    robot_model_list.insert(robot_model_list.end(),
      entities.begin(), entities.end());
  }

  Entity robot_model;
  if (!find_nearest_model(ecm, robot_model_list, robot_model))
  {
    RCLCPP_WARN(_dispenser_common->ros_node->get_logger(),
      "No nearby robots of fleet [%s] found.", fleet_name.c_str());
    return false;
  }
  place_on_entity(ecm, robot_model, _item_en);
  _dispenser_common->dispenser_filled = false; // Assumes Dispenser is configured to only dispense a single object
  return true;
}

// Searches vicinity of Dispenser for closest valid item. If found, _item_en is set to the item's entity
void TeleportDispenserPlugin::fill_dispenser(EntityComponentManager& ecm)
{
  const auto dispenser_pos =
    ecm.Component<components::Pose>(_dispenser)->Data().Pos();

  double nearest_dist = 1.0;
  ecm.Each<components::Model, components::Name, components::Pose,
    components::Static>(
    [&](const Entity& en,
    const components::Model*,
    const components::Name* name,
    const components::Pose* pose,
    const components::Static* is_static
    ) -> bool
    {
      if (!is_static->Data() && name->Data() != _dispenser_common->guid)
      {
        const auto dist = pose->Data().Pos().Distance(dispenser_pos);

        if (dist < nearest_dist
        && ecm.Component<components::AxisAlignedBox>(_dispenser)->Data().
        Contains(pose->Data().Pos()))
        {
          _item_en = en;
          nearest_dist = dist;
          _dispenser_common->dispenser_filled = true;
          _dispenser_common->item_en_found = true;
        }
      }
      return true;
    });

  if (!_dispenser_common->dispenser_filled)
  {
    RCLCPP_WARN(_dispenser_common->ros_node->get_logger(),
      "Could not find dispenser item model within 1 meter, "
      "this dispenser will not be operational");
  }
  else
  {
    RCLCPP_INFO(_dispenser_common->ros_node->get_logger(),
      "Found dispenser item: [%s]",
      ecm.Component<components::Name>(_item_en)->Data().c_str());

    // Create Bounding Box component to enable dispensing item later
    if (!ecm.EntityHasComponentType(_item_en,
      components::AxisAlignedBox().TypeId()))
    {
      ecm.CreateComponent(_item_en, components::AxisAlignedBox());
    }
  }
}

void TeleportDispenserPlugin::create_dispenser_bounding_box(
  EntityComponentManager& ecm)
{
  const auto dispenser_pos =
    ecm.Component<components::Pose>(_dispenser)->Data().Pos();
  ignition::math::Vector3d corner_1(dispenser_pos.X() - 0.05,
    dispenser_pos.Y() - 0.05, dispenser_pos.Z() - 0.05);
  ignition::math::Vector3d corner_2(dispenser_pos.X() + 0.05,
    dispenser_pos.Y() + 0.05, dispenser_pos.Z() + 0.05);
  _dispenser_vicinity_box = ignition::math::AxisAlignedBox(corner_1, corner_2);

  if (!ecm.EntityHasComponentType(_dispenser,
    components::AxisAlignedBox().TypeId()))
  {
    ecm.CreateComponent(_dispenser,
      components::AxisAlignedBox(_dispenser_vicinity_box));
  }
  else
  {
    ecm.Component<components::AxisAlignedBox>(_dispenser)->Data() =
      _dispenser_vicinity_box;
  }
}

void TeleportDispenserPlugin::Configure(const Entity& entity,
  const std::shared_ptr<const sdf::Element>&,
  EntityComponentManager& ecm, EventManager&)
{
  char const** argv = NULL;
  if (!rclcpp::is_initialized())
    rclcpp::init(0, argv);

  _dispenser = entity;
  _dispenser_common->guid =
    ecm.Component<components::Name>(_dispenser)->Data();
  ignwarn << "Initializing plugin with name " << _dispenser_common->guid <<
    std::endl;

  _ros_node = std::make_shared<rclcpp::Node>(_dispenser_common->guid);
  _dispenser_common->init_ros_node(_ros_node);
  RCLCPP_INFO(_dispenser_common->ros_node->get_logger(), "Started node...");

  create_dispenser_bounding_box(ecm);
}

void TeleportDispenserPlugin::PreUpdate(const UpdateInfo& info,
  EntityComponentManager& ecm)
{
  _dispenser_common->sim_time =
    std::chrono::duration_cast<std::chrono::seconds>(info.simTime).count();
  // TODO parallel thread executor?
  rclcpp::spin_some(_dispenser_common->ros_node);

  // Set item that the Dispenser will be configured to dispense. Do this only on first PreUpdate() call.
  // Happens here and not in Configure() to allow for all models to load
  if (!tried_fill_dispenser)
  {
    fill_dispenser(ecm);
    tried_fill_dispenser = true;
  }

  std::function<bool(const std::string&)> dispense_onto_robot_cb =
    std::bind(&TeleportDispenserPlugin::dispense_on_nearest_robot,
      this, std::ref(ecm), std::placeholders::_1);

  std::function<bool(void)> check_filled_cb = [&]()
    {
      return ecm.Component<components::AxisAlignedBox>(_dispenser)->Data().
        Contains(ecm.Component<components::Pose>(_item_en)->Data().Pos());
    };

  _dispenser_common->on_update(dispense_onto_robot_cb, check_filled_cb);
}

IGNITION_ADD_PLUGIN(
  TeleportDispenserPlugin,
  System,
  TeleportDispenserPlugin::ISystemConfigure,
  TeleportDispenserPlugin::ISystemPreUpdate)

// TODO would prefer namespaced
IGNITION_ADD_PLUGIN_ALIAS(TeleportDispenserPlugin, "teleport_dispenser")

} // namespace rmf_ignition_plugins