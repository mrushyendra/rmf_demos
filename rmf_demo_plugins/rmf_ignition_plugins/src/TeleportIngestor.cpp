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

#include <rmf_plugins_common/ingestor_common.hpp>

// TODO remove this
using namespace ignition;
using namespace ignition::gazebo;
using namespace rmf_ingestor_common;

namespace rmf_ignition_plugins {

class IGNITION_GAZEBO_VISIBLE TeleportIngestorPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
public:

  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;

  TeleportIngestorPlugin();
  ~TeleportIngestorPlugin();
  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>&,
    EntityComponentManager& ecm, EventManager&) override;
  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override;

private:
  std::unique_ptr<TeleportIngestorCommon> IngestorCommonPtr;

  Entity _ingestor;
  Entity _ingested_entity;
  bool _load_complete = false;

  rclcpp::Node::SharedPtr _ros_node;

  std::unordered_map<std::string, ignition::math::Pose3d>
  _non_static_models_init_poses;

  bool find_nearest_non_static_model(
    const EntityComponentManager& ecm,
    const std::vector<Entity>& robot_model_entities,
    Entity& robot_entity) const;
  bool get_payload_model(
    const EntityComponentManager& ecm,
    const Entity& robot_entity,
    Entity& payload_entity);
  void ingest_from_nearest_robot(EntityComponentManager& ecm, const std::string& fleet_name);
  void send_ingested_item_home(EntityComponentManager& ecm);
};

bool TeleportIngestorPlugin::find_nearest_non_static_model(
  const EntityComponentManager& ecm,
  const std::vector<Entity>& robot_model_entities,
  Entity& robot_entity) const
{
  double nearest_dist = 1e6;
  bool found = false;
  const auto ingestor_pos = ecm.Component<components::Pose>(_ingestor)->Data().Pos();

  for (const auto& en : robot_model_entities)
  {
    bool is_static = ecm.Component<components::Static>(en)->Data();
    std::string name = ecm.Component<components::Name>(en)->Data();

    if (!en || is_static || name == IngestorCommonPtr->_guid)
      continue;

    const auto en_pos = ecm.Component<components::Pose>(en)->Data().Pos();
    const double dist = en_pos.Distance(ingestor_pos);
    if (dist < nearest_dist)
    {
      nearest_dist = dist;
      robot_entity = en;
      found = true;
    }
  }
  return found;
}

bool TeleportIngestorPlugin::get_payload_model(
  const EntityComponentManager& ecm,
  const Entity& robot_entity,
  Entity& payload_entity)
{
  if(ecm.Component<components::AxisAlignedBox>(robot_entity)){
    std::cout << "Bounding box: " << ecm.Component<components::AxisAlignedBox>(robot_entity)->Data() << std::endl;
  }
  //const ignition::math::Box robot_collision_bb = ecm.Component<components::BoundingBox>(robot_entity)->Data(); //robot_model->BoundingBox();
  //ignition::math::Vector3d max_corner = robot_collision_bb.Max(); //

  // create a new bounding box extended slightly in the Z direction
  //max_corner.Z(max_corner.Z() + 0.1);
  //const ignition::math::Box vicinity_box(robot_collision_bb.Min(), max_corner);

  // There might not be a better way to loop through all the models, as we
  // might consider delivering items that were spawned during run time,
  // instead of during launch.
  const auto robot_model_pos = ecm.Component<components::Pose>(robot_entity)->Data().Pos(); //need world pose
  double nearest_dist = 1.0; //what is this number
  bool found = false;

  ecm.Each<components::Model, components::Name, components::Pose,
    components::Static>( //to check Static param
    [&](const Entity& entity,
    const components::Model*,
    const components::Name* name,
    const components::Pose* pose,
    const components::Static* is_static
    ) -> bool
    {
      if (!is_static->Data() && name->Data() != IngestorCommonPtr->_guid
      && name->Data() != Model(robot_entity).Name(ecm))
      {
        const double dist = pose->Data().Pos().Distance(robot_model_pos);
        if (dist < nearest_dist) //&& vicinity_box.Intersects(model->BoundingBox()))//
        {
          payload_entity = entity;
          nearest_dist = dist;
          found = true;
        }
      }
      return true;
    });

  return found;
}

void TeleportIngestorPlugin::ingest_from_nearest_robot(EntityComponentManager& ecm, const std::string& fleet_name)
{
  const auto fleet_state_it = IngestorCommonPtr->_fleet_states.find(fleet_name);
  if (fleet_state_it == IngestorCommonPtr->_fleet_states.end())
  {
    RCLCPP_WARN(IngestorCommonPtr->_ros_node->get_logger(),
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
    RCLCPP_WARN(IngestorCommonPtr->_ros_node->get_logger(),
      "No nearby robots of fleet [%s] found.", fleet_name.c_str());
    return;
  }

  if (!get_payload_model(ecm, robot_model, _ingested_entity))
  {
    RCLCPP_WARN(IngestorCommonPtr->_ros_node->get_logger(),
      "No delivery item found on the robot: [%s]",
      Model(robot_model).Name(ecm));
    return;
  }

  auto cmd = ecm.Component<components::WorldPoseCmd>(_ingested_entity);
  if (!cmd) {
    ecm.CreateComponent(_ingested_entity, components::WorldPoseCmd(ignition::math::Pose3<double>()));
  }
  auto new_pose = ecm.Component<components::Pose>(_ingestor)->Data()+ ignition::math::Pose3<double>(0,0,0.5,0,0,0);;
  ecm.Component<components::WorldPoseCmd>(_ingested_entity)->Data() = new_pose;
  IngestorCommonPtr->_ingestor_filled = true;
}

void TeleportIngestorPlugin::send_ingested_item_home(EntityComponentManager& ecm)
{
  if (IngestorCommonPtr->_ingestor_filled)
  {
    const auto it = _non_static_models_init_poses.find(Model(_ingested_entity).Name(ecm));
    if (it == _non_static_models_init_poses.end()) {
      ecm.RequestRemoveEntity(_ingested_entity);
    } else {
      auto cmd = ecm.Component<components::WorldPoseCmd>(_ingested_entity);
      if (!cmd) {
        ecm.CreateComponent(_ingested_entity, components::WorldPoseCmd(ignition::math::Pose3<double>()));
      }
      ecm.Component<components::WorldPoseCmd>(_ingested_entity)->Data() = it->second;
    }
    IngestorCommonPtr->_ingestor_filled = false;
  }
}

TeleportIngestorPlugin::TeleportIngestorPlugin()
: IngestorCommonPtr(std::make_unique<TeleportIngestorCommon>())
{
  // We do initialization only during ::Configure
}

TeleportIngestorPlugin::~TeleportIngestorPlugin()
{
}

void TeleportIngestorPlugin::Configure(const Entity& entity,
  const std::shared_ptr<const sdf::Element>&,
  EntityComponentManager& ecm, EventManager&)
{
  char const** argv = NULL;
  if (!rclcpp::is_initialized())
    rclcpp::init(0, argv);

  _ingestor = entity;
  IngestorCommonPtr->_guid = ecm.Component<components::Name>(_ingestor)->Data();
  std::string plugin_name("plugin_" + IngestorCommonPtr->_guid);
  ignwarn << "Initializing plugin with name " << plugin_name << std::endl;
  _ros_node = std::make_shared<rclcpp::Node>(plugin_name);
  IngestorCommonPtr->init_ros_node(_ros_node);
  RCLCPP_INFO(IngestorCommonPtr->_ros_node->get_logger(), "Started node...");

  // Keep track of all the non-static models
  ecm.Each<components::Model, components::Name, components::Pose, components::Static>(
    [&](const Entity&,
    const components::Model*,
    const components::Name* name,
    const components::Pose* pose,
    const components::Static* is_static
    ) -> bool
    {
      if (!is_static->Data() && name->Data() != IngestorCommonPtr->_guid)
      {
        _non_static_models_init_poses[name->Data()] = pose->Data();
      }
      return true;
    });

  IngestorCommonPtr->_current_state.guid = IngestorCommonPtr->_guid;
  IngestorCommonPtr->_current_state.mode = DispenserState::IDLE;

  _load_complete = true;
}

void TeleportIngestorPlugin::PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm)
{
  IngestorCommonPtr->_sim_time = (std::chrono::duration_cast<std::chrono::seconds>(info.simTime).count());

  // TODO parallel thread executor?
  rclcpp::spin_some(IngestorCommonPtr->_ros_node);
  if (!_load_complete) {
    return;
  }

  if(IngestorCommonPtr->_ingest) { // Only ingests max once per call to PreUpdate()
    IngestorCommonPtr->send_ingestor_response(DispenserResult::ACKNOWLEDGED);

    RCLCPP_INFO(IngestorCommonPtr->_ros_node->get_logger(), "Ingesting item");
    ingest_from_nearest_robot(ecm, IngestorCommonPtr->latest.transporter_type);

    IngestorCommonPtr->send_ingestor_response(DispenserResult::SUCCESS);

    //rclcpp::sleep_for(std::chrono::seconds(10));
    //send_ingested_item_home(ecm);
    IngestorCommonPtr->_last_ingested_time = IngestorCommonPtr->_sim_time;
    IngestorCommonPtr->_ingest= false;
  }

  if (IngestorCommonPtr->_sim_time - IngestorCommonPtr->_last_pub_time >= 2.0)
  {
    IngestorCommonPtr->_last_pub_time = IngestorCommonPtr->_sim_time;
    const auto now = IngestorCommonPtr->simulation_now(IngestorCommonPtr->_sim_time);

    IngestorCommonPtr->_current_state.time = now;
    IngestorCommonPtr->_current_state.mode = DispenserState::IDLE;
    IngestorCommonPtr->_state_pub->publish(IngestorCommonPtr->_current_state);
  }

  if(IngestorCommonPtr->_sim_time - IngestorCommonPtr->_last_ingested_time >= 5.0 && IngestorCommonPtr->_ingestor_filled){
    send_ingested_item_home(ecm); 
  }
}

IGNITION_ADD_PLUGIN(
  TeleportIngestorPlugin,
  System,
  TeleportIngestorPlugin::ISystemConfigure,
  TeleportIngestorPlugin::ISystemPreUpdate)

// TODO would prefer namespaced
IGNITION_ADD_PLUGIN_ALIAS(TeleportIngestorPlugin, "teleport_ingestor")

} // namespace rmf_ignition_plugins

