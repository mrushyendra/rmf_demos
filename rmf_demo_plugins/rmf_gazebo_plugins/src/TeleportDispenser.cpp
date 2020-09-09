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

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <ignition/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>

#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>

#include <rmf_plugins_common/dispenser_common.hpp>
#include <rmf_plugins_common/utils.hpp>

using namespace rmf_dispenser_common;
using namespace rmf_plugins_utils;

namespace rmf_gazebo_plugins {

class TeleportDispenserPlugin : public gazebo::ModelPlugin
{

public:

  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;

  TeleportDispenserPlugin();
  ~TeleportDispenserPlugin();
  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

private:
  // Stores params representing state of Dispenser, and handles all message pub/sub
  std::unique_ptr<TeleportDispenserCommon> _dispenser_common;

  gazebo::event::ConnectionPtr _update_connection;
  gazebo::physics::ModelPtr _model;
  gazebo::physics::ModelPtr _item_model; // Item that dispenser may contain
  gazebo::physics::WorldPtr _world;

  #if GAZEBO_MAJOR_VERSION <= 9
  ignition::math::Box _dispenser_vicinity_box;
  #else
  ignition::math::AxisAlignedBox _dispenser_vicinity_box;
  #endif

  bool find_nearest_model_name(
    const std::vector<gazebo::physics::ModelPtr>& models,
    std::string& nearest_model_name) const;
  bool dispense_on_nearest_robot(const std::string& fleet_name);
  void fill_dispenser();
  void create_dispenser_bounding_box();
  void on_update();
};

bool TeleportDispenserPlugin::find_nearest_model_name(
  const std::vector<gazebo::physics::ModelPtr>& models,
  std::string& nearest_model_name) const
{
  double nearest_dist = 1e6;
  bool found = false;

  for (const auto& m : models)
  {
    if (!m || m->GetName() == _dispenser_common->guid)
      continue;

    const double dist =
      m->WorldPose().Pos().Distance(_model->WorldPose().Pos());
    if (dist < nearest_dist)
    {
      nearest_dist = dist;
      nearest_model_name = m->GetName();
      found = true;
    }
  }
  return found;
}

bool TeleportDispenserPlugin::dispense_on_nearest_robot(
  const std::string& fleet_name)
{
  if (!_item_model)
    return false;

  const auto fleet_state_it = _dispenser_common->fleet_states.find(fleet_name);
  if (fleet_state_it == _dispenser_common->fleet_states.end())
  {
    RCLCPP_WARN(_dispenser_common->ros_node->get_logger(),
      "No such fleet: [%s]", fleet_name.c_str());
    return false;
  }

  std::vector<gazebo::physics::ModelPtr> robot_models;
  for (const auto& rs : fleet_state_it->second->robots)
  {
    const auto r_model = _world->ModelByName(rs.name);
    if (r_model && !r_model->IsStatic())
      robot_models.push_back(r_model);
  }

  std::string nearest_robot_model_name;
  if (!find_nearest_model_name(
      robot_models, nearest_robot_model_name))
  {
    RCLCPP_WARN(_dispenser_common->ros_node->get_logger(),
      "No near robots of fleet [%s] found.", fleet_name.c_str());
    return false;
  }
  _item_model->PlaceOnEntity(nearest_robot_model_name);
  _dispenser_common->dispenser_filled = false;
  return true;
}

// Searches vicinity of Dispenser for closest valid item. If found, _item_model is set to the newly found item
void TeleportDispenserPlugin::fill_dispenser()
{
  auto model_list = _world->Models();
  double nearest_dist = 1.0;
  const auto dispenser_pos = _model->WorldPose().Pos();
  for (const auto& m : model_list)
  {
    if (!m || m->IsStatic() || m->GetName() == _model->GetName())
      continue;

    const double dist = m->WorldPose().Pos().Distance(dispenser_pos);
    if (dist < nearest_dist &&
      _dispenser_vicinity_box.Intersects(m->BoundingBox()))
    {
      _item_model = m;
      nearest_dist = dist;
      _dispenser_common->dispenser_filled = true;
    }
  }
}

void TeleportDispenserPlugin::create_dispenser_bounding_box()
{
  // Find the dispenser item model, maximum distance 1 meter
  const auto dispenser_pos = _model->WorldPose().Pos();
  ignition::math::Vector3d corner_1 = dispenser_pos;
  corner_1.X(dispenser_pos.X() - 0.05);
  corner_1.Y(dispenser_pos.Y() - 0.05);
  corner_1.Z(dispenser_pos.Z() - 0.05);
  ignition::math::Vector3d corner_2 = dispenser_pos;
  corner_2.X(dispenser_pos.X() + 0.05);
  corner_2.Y(dispenser_pos.Y() + 0.05);
  corner_2.Z(dispenser_pos.Z() + 0.05);
  #if GAZEBO_MAJOR_VERSION <= 9
  _dispenser_vicinity_box = ignition::math::Box(corner_1, corner_2);
  #else
  _dispenser_vicinity_box =
    ignition::math::AxisAlignedBox(corner_1, corner_2);
  #endif
}

void TeleportDispenserPlugin::on_update()
{
  _dispenser_common->sim_time = _world->SimTime().Double();

  std::function<bool(const std::string&)> dispense_onto_robot_cb =
    std::bind(&TeleportDispenserPlugin::dispense_on_nearest_robot,
      this, std::placeholders::_1);
  _dispenser_common->on_update(dispense_onto_robot_cb);

  const double t = _world->SimTime().Double();
  constexpr double interval = 2.0;
  if (t - _dispenser_common->last_pub_time >= interval)
  {
    // Occasionally check to see if dispensed item has been returned to it
    if (!_dispenser_common->dispenser_filled &&
      _item_model &&
      _model->BoundingBox().Intersects(
        _item_model->BoundingBox()))
      _dispenser_common->dispenser_filled = true;
  }
}

TeleportDispenserPlugin::TeleportDispenserPlugin()
: _dispenser_common(std::make_unique<TeleportDispenserCommon>())
{
}

TeleportDispenserPlugin::~TeleportDispenserPlugin()
{
  rclcpp::shutdown();
}

void TeleportDispenserPlugin::Load(gazebo::physics::ModelPtr _parent,
  sdf::ElementPtr _sdf)
{
  _model = _parent;
  _world = _model->GetWorld();
  _dispenser_common->guid = _model->GetName();

  _dispenser_common->init_ros_node(gazebo_ros::Node::Get(_sdf));
  RCLCPP_INFO(
    _dispenser_common->ros_node->get_logger(),
    "Started TeleportDispenserPlugin node...");

  create_dispenser_bounding_box();
  fill_dispenser();

  if (!_item_model)
  {
    RCLCPP_WARN(_dispenser_common->ros_node->get_logger(),
      "Could not find dispenser item model within 1 meter, "
      "this dispenser will not be operational");
    return;
  }

  RCLCPP_INFO(_dispenser_common->ros_node->get_logger(),
    "Found dispenser item: [%s]", _item_model->GetName().c_str());

  _dispenser_common->current_state.guid = _dispenser_common->guid;
  _dispenser_common->current_state.mode = DispenserState::IDLE;

  _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&TeleportDispenserPlugin::on_update, this));
}

} // namespace rmf_gazebo_plugins

GZ_REGISTER_MODEL_PLUGIN(rmf_gazebo_plugins::TeleportDispenserPlugin)
