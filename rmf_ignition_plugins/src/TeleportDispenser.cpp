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
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include <sdf/Geometry.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/AxisAlignedBox.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Box.hh>
#include <ignition/math/AxisAlignedBox.hh>

#include <rclcpp/rclcpp.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>

// TODO remove this
using namespace ignition;
using namespace ignition::gazebo;

namespace rmf_ignition_plugins {

class IGNITION_GAZEBO_VISIBLE TeleportDispenserPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
public:

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
  using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;
  using Pose3d = ignition::math::Pose3d;

private:
  // Dispense request params
  bool _dispense = false;
  DispenserRequest latest;
  std::string _request_guid;
  std::string _target_guid;
  std::string _transporter_type;

  // General params
  std::string _guid;
  double _last_pub_time = 0.0;
  bool _initialized = false;
  Entity _dispenser;
  Entity _item_en;
  bool _item_en_found = false;
  bool _item_dispensed = false;
  bool _try_fill_dispenser = false;
  ignition::math::AxisAlignedBox _dispenser_vicinity_box;

  rclcpp::Node::SharedPtr _ros_node;
  rclcpp::Subscription<FleetState>::SharedPtr _fleet_state_sub;
  rclcpp::Publisher<DispenserState>::SharedPtr _state_pub;
  rclcpp::Subscription<DispenserRequest>::SharedPtr _request_sub;
  rclcpp::Publisher<DispenserResult>::SharedPtr _result_pub;

  std::unordered_map<std::string, FleetState::UniquePtr> _fleet_states;

  std::unordered_map<std::string, bool> _past_request_guids;

  DispenserState _current_state;

  std::chrono::steady_clock::duration _sim_time;

  rclcpp::Time simulation_now(const double t) const //common
  {
    const int32_t t_sec = static_cast<int32_t>(t);
    const uint32_t t_nsec =
      static_cast<uint32_t>((t-static_cast<double>(t_sec)) * 1e9);
    return rclcpp::Time{t_sec, t_nsec, RCL_ROS_TIME};
  }

  bool find_nearest_non_static_model(
    EntityComponentManager& ecm,
    const std::vector<Entity>& robot_model_entities,
    Entity& robot_entity) const
  {
    double nearest_dist = 1e6;
    bool found = false;
    const auto dispenser_pos = ecm.Component<components::Pose>(_dispenser)->Data().Pos();

    for (const auto& en : robot_model_entities)
    {
      bool is_static = ecm.Component<components::Static>(en)->Data();
      std::string name = ecm.Component<components::Name>(en)->Data();

      if (!en || is_static || name == _guid)
        continue;

      const auto en_pos = ecm.Component<components::Pose>(en)->Data().Pos();
      const double dist = en_pos.Distance(dispenser_pos); //need to get world pose
      if (dist < nearest_dist) //to be common, maybe even earlier static part
      {
        nearest_dist = dist;
        robot_entity = en;
        found = true;
      }
    }
    return found;
  }

  void dispense_on_nearest_robot(EntityComponentManager& ecm, const std::string& fleet_name) const
  {
    if (!_item_en_found)
      return;

    const auto fleet_state_it = _fleet_states.find(fleet_name);
    if (fleet_state_it == _fleet_states.end())
    {
      RCLCPP_WARN(_ros_node->get_logger(),
        "No such fleet: [%s]", fleet_name.c_str());
      return;
    }

    std::vector<Entity> robot_model_list;
    for (const auto& rs : fleet_state_it->second->robots)
    {
      std::vector<Entity> entities = ecm.EntitiesByComponents(components::Name(rs.name), components::Model());
      robot_model_list.insert(robot_model_list.end(), entities.begin(), entities.end());
    }

    Entity robot_model;
    if (!find_nearest_non_static_model(ecm, robot_model_list, robot_model))
    {
      RCLCPP_WARN(_ros_node->get_logger(),
        "No nearby robots of fleet [%s] found.", fleet_name.c_str());
      return;
    }

    auto cmd = ecm.Component<components::WorldPoseCmd>(_item_en);
    if (!cmd) {
      auto pose = ignition::math::Pose3<double>();
      ecm.CreateComponent(_item_en, components::WorldPoseCmd(pose));
    }
    auto new_pose = ecm.Component<components::Pose>(robot_model)->Data() += ignition::math::Pose3<double>(0,0,1,0,0,0);
    ecm.Component<components::WorldPoseCmd>(_item_en)->Data() = new_pose;
  }

  void fleet_state_cb(FleetState::UniquePtr msg)
  {
    _fleet_states[msg->name] = std::move(msg);
  }

  void send_dispenser_response(uint8_t status) const
  {
    DispenserResult response;
    response.time = simulation_now(std::chrono::duration_cast<std::chrono::nanoseconds>(_sim_time).count() * 1e-9); //maybe there's a better way to do this
    response.request_guid = _request_guid;
    response.source_guid = _guid;
    response.status = status;
    _result_pub->publish(response);
  }

  void dispenser_request_cb(DispenserRequest::UniquePtr msg)
  {
    latest = *msg;
    _transporter_type = msg->transporter_type;
    _request_guid = msg->request_guid;
    _target_guid = msg->target_guid;

    if (_guid == latest.target_guid && !_item_dispensed)
    {
      const auto it = _past_request_guids.find(latest.request_guid);
      if (it != _past_request_guids.end())
      {
        if (it->second)
        {
          RCLCPP_WARN(_ros_node->get_logger(),
            "Request already succeeded: [%s]", latest.request_guid);
          send_dispenser_response(DispenserResult::SUCCESS);
        }
        else
        {
          RCLCPP_WARN(_ros_node->get_logger(),
            "Request already failed: [%s]", latest.request_guid);
          send_dispenser_response(DispenserResult::FAILED);
        }
        return;
      }

      _dispense = true; // mark true to dispense item next time PreUpdate() is called
      // There are currently no cases to publish a FAILED result yet
    }
  }

  void fill_dispenser(EntityComponentManager& ecm){
      const auto dispenser_pos = ecm.Component<components::Pose>(_dispenser)->Data().Pos();
      //_dispenser_vicinity_box = ignition::math::Box<double>(0.1,0.1,0.1);

      double nearest_dist = 1.0;
      ecm.Each<components::Model, components::Name, components::Pose, components::Static>(
        [&](const Entity& en,
        const components::Model*,
        const components::Name* name,
        const components::Pose* pose,
        const components::Static* is_static
        ) -> bool
        {
          if (!is_static->Data() && name->Data() != _guid)
          {
            const auto dist = pose->Data().Pos().Distance(dispenser_pos);
            if (dist < nearest_dist) //&& _dispenser_vicinity_box.Intersects(m->BoundingBox()))
            {
              _item_en = en;
              _item_en_found = true;
              nearest_dist = dist;

              /* Link link(ecm.EntityByComponents(
                components::ParentEntity(en),
                components::Name("body"),
                components::Link()));

              std::cout << "Name: " << *link.Name(ecm) << std::endl;
              std::cout << "Pose: " << link.WorldPose(ecm)->Pos() << std::endl;
              */
            }
          }
          return true;
        });

      if (!_item_en_found)
      {
        RCLCPP_WARN(_ros_node->get_logger(),
          "Could not find dispenser item model within 1 meter, "
          "this dispenser will not be operational");
      } else {
        RCLCPP_INFO(_ros_node->get_logger(),
          "Found dispenser item: [%s]", ecm.Component<components::Name>(_item_en)->Data().c_str());
      }
      _try_fill_dispenser = true;
  }

public:

  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>&,
    EntityComponentManager& ecm, EventManager&) override
  {
    char const** argv = NULL;
    if (!rclcpp::is_initialized())
      rclcpp::init(0, argv);

    _dispenser = entity;
    _guid = ecm.Component<components::Name>(_dispenser)->Data();
    std::string plugin_name("plugin_" + _guid);
    ignwarn << "Initializing plugin with name " << plugin_name << std::endl;
    _ros_node = std::make_shared<rclcpp::Node>(plugin_name);
    RCLCPP_INFO(_ros_node->get_logger(), "Started node...");
    
    // create a bounding box for the dispenser
    const auto dispenser_pos =  ecm.Component<components::Pose>(_dispenser)->Data().Pos();
    ignition::math::Vector3d corner_1(dispenser_pos.X() - 0.05, dispenser_pos.Y() - 0.05, dispenser_pos.Z() - 0.05);
    ignition::math::Vector3d corner_2(dispenser_pos.X() + 0.05, dispenser_pos.Y() + 0.05, dispenser_pos.Z() + 0.05);
    _dispenser_vicinity_box = ignition::math::AxisAlignedBox(corner_1, corner_2);
    auto aabb = ecm.Component<components::AxisAlignedBox>(_dispenser);
    if (!aabb) {
      ecm.CreateComponent(_dispenser, components::AxisAlignedBox(_dispenser_vicinity_box));
    } else {
      ecm.Component<components::AxisAlignedBox>(_dispenser)->Data() = _dispenser_vicinity_box;
    }

    _fleet_state_sub = _ros_node->create_subscription<FleetState>(
      "/fleet_states",
      rclcpp::SystemDefaultsQoS(),
      [&](FleetState::UniquePtr msg)
      {
        fleet_state_cb(std::move(msg));
      });

    _state_pub = _ros_node->create_publisher<DispenserState>(
      "/dispenser_states", 10);

    _request_sub = _ros_node->create_subscription<DispenserRequest>(
      "/dispenser_requests",
      rclcpp::SystemDefaultsQoS(),
      [&](DispenserRequest::UniquePtr msg)
      {
        dispenser_request_cb(std::move(msg));
      });

    _result_pub = _ros_node->create_publisher<DispenserResult>(
      "/dispenser_results", 10);

    _current_state.guid = _guid;
    _current_state.mode = DispenserState::IDLE;

    _initialized = true;
  }

  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override
  {
    _sim_time = info.simTime;
    // TODO parallel thread executor?
    rclcpp::spin_some(_ros_node);
    if (!_initialized) {
      return;
    }

    if(!_try_fill_dispenser){
      fill_dispenser(ecm);
    }

    if(_dispense){ // Only dispenses max once per call to PreUpdate()
      send_dispenser_response(DispenserResult::ACKNOWLEDGED);

      RCLCPP_INFO(_ros_node->get_logger(), "Dispensing item");
      dispense_on_nearest_robot(ecm, _transporter_type);
      rclcpp::sleep_for(std::chrono::seconds(5));

      _item_dispensed = true;
      send_dispenser_response(DispenserResult::SUCCESS);
      _item_en_found = false;
      _dispense = false;
    }

    double t =
      (std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).
      count()) * 1e-9;

    if (t - _last_pub_time >= 2.0)
    {
      _last_pub_time = t;
      const auto now = simulation_now(t);

      _current_state.time = now;
      _current_state.mode = DispenserState::IDLE;
      _state_pub->publish(_current_state);

      if (_item_dispensed &&
        _item_en_found /*&&
        _model->BoundingBox().Intersects(
          _item_model->BoundingBox())*/)
        _item_dispensed = false;
    }
  }
};

IGNITION_ADD_PLUGIN(
  TeleportDispenserPlugin,
  System,
  TeleportDispenserPlugin::ISystemConfigure,
  TeleportDispenserPlugin::ISystemPreUpdate)

// TODO would prefer namespaced
IGNITION_ADD_PLUGIN_ALIAS(TeleportDispenserPlugin, "teleport_dispenser")

} // namespace rmf_ignition_plugins

