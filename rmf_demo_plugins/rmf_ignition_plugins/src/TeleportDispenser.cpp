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
#include <unordered_map>

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
#include <rmf_fleet_msgs/msg/fleet_state.hpp>

#include <rmf_plugins_common/dispenser_common.hpp>
#include <rmf_plugins_common/utils.hpp>

using namespace ignition::gazebo;
using namespace rmf_dispenser_common;
using namespace rmf_plugins_utils;

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

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using FleetStateIt =
    std::unordered_map<std::string, FleetState::UniquePtr>::iterator;

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
    const std::vector<SimEntity>& entities, SimEntity& sim_obj) const;
  void place_on_entity(EntityComponentManager& ecm,
    const SimEntity& obj, const Entity& to_move);
  void fill_robot_model_list(EntityComponentManager& ecm,
    FleetStateIt fleet_state_it, std::vector<SimEntity>& robot_model_list);
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
  const std::vector<SimEntity>& entities,
  SimEntity& robot_entity) const
{
  double nearest_dist = 1e6;
  bool found = false;
  const auto dispenser_pos =
    ecm.Component<components::Pose>(_dispenser)->Data().Pos();

  for (const auto& sim_obj : entities)
  {
    Entity en = sim_obj.entity;
    std::string name = ecm.Component<components::Name>(en)->Data();
    if (name == _dispenser_common->guid)
      continue;

    const auto en_pos = ecm.Component<components::Pose>(en)->Data().Pos();
    const double dist = en_pos.Distance(dispenser_pos);
    if (dist < nearest_dist)
    {
      nearest_dist = dist;
      robot_entity = sim_obj;
      found = true;
    }
  }
  return found;
}

// Move entity `to_move` onto `base`
void TeleportDispenserPlugin::place_on_entity(EntityComponentManager& ecm,
  const SimEntity& base_obj, const Entity& to_move)
{
  Entity base = base_obj.entity;
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

void TeleportDispenserPlugin::fill_robot_model_list(EntityComponentManager& ecm,
  FleetStateIt fleet_state_it, std::vector<SimEntity>& robot_model_list)
{
  for (const auto& rs : fleet_state_it->second->robots)
  {
    std::vector<Entity> entities =
      ecm.EntitiesByComponents(components::Name(rs.name),
        components::Model(), components::Static(false));
    for (Entity& en : entities)
    {
      robot_model_list.push_back(SimEntity(en));
    }
  }
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
  RCLCPP_INFO(_dispenser_common->ros_node->get_logger(),
    "Started TeleportIngestorPlugin node...");

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

  std::function<void(FleetStateIt,
    std::vector<rmf_plugins_utils::SimEntity>&)> fill_robot_model_list_cb =
    std::bind(&TeleportDispenserPlugin::fill_robot_model_list, this,
      std::ref(ecm), std::placeholders::_1, std::placeholders::_2);

  std::function<bool(const std::vector<rmf_plugins_utils::SimEntity>&,
    SimEntity&)> find_nearest_model_cb =
    std::bind(&TeleportDispenserPlugin::find_nearest_model, this,
      std::ref(ecm), std::placeholders::_1, std::placeholders::_2);

  std::function<void(const SimEntity&)> place_on_entity_cb =
    std::bind(&TeleportDispenserPlugin::place_on_entity, this,
      std::ref(ecm), std::placeholders::_1, _item_en);

  std::function<bool(void)> check_filled_cb = [&]()
    {
      return ecm.Component<components::AxisAlignedBox>(_dispenser)->Data().
        Contains(ecm.Component<components::Pose>(_item_en)->Data().Pos());
    };

  _dispenser_common->on_update(fill_robot_model_list_cb, find_nearest_model_cb,
    place_on_entity_cb, check_filled_cb);
}

IGNITION_ADD_PLUGIN(
  TeleportDispenserPlugin,
  System,
  TeleportDispenserPlugin::ISystemConfigure,
  TeleportDispenserPlugin::ISystemPreUpdate)

// TODO would prefer namespaced
IGNITION_ADD_PLUGIN_ALIAS(TeleportDispenserPlugin, "teleport_dispenser")

} // namespace rmf_ignition_plugins