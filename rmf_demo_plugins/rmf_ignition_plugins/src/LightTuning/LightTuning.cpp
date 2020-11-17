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

#include <string>
#include <iostream>

#include <ignition/plugin/Register.hh>
#include <ignition/gui/qt.h>
//  #include <ignition/gui/Plugin.hh>
#include <ignition/gazebo/gui/GuiSystem.hh>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

using namespace ignition;
//using namespace gui;

class LightTuning : public gazebo::GuiSystem
{
  Q_OBJECT

private:
  bool _light_on = false;
  ignition::transport::Node _node;

  void checkbox_checked(const std::string& name, bool checked);

protected slots:
  void OnEnableLight(bool);

public:
  LightTuning();

  virtual void LoadConfig(const tinyxml2::XMLElement* _pluginElem)
  override;

  void Update(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm) override;
};

LightTuning::LightTuning()
{
  /*_charge_state_pub = _node.Advertise<ignition::msgs::Selection>(
    "/charge_state");
  if (!_charge_state_pub)
  {
    std::cerr << "Error advertising topic [/charge_state]" << std::endl;
  }*/
}

void LightTuning::LoadConfig(const tinyxml2::XMLElement* _pluginElem)
{
  if (!_pluginElem)
    return;

  if (this->title.empty())
    this->title = "Light Tuning";
}

void LightTuning::Update(const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  return;
}

void LightTuning::OnEnableLight(bool checked)
{
  _light_on = checked;
  checkbox_checked("Light On", checked);
}

void LightTuning::checkbox_checked(
  const std::string& name, bool checked)
{
  ignition::msgs::EntityFactory create_light;
  create_light.set_sdf(
  "<sdf version=\"1.7\">\
    <light type=\"directional\" name=\"sun\">\
    <cast_shadows>true</cast_shadows>\
    <pose>0 0 10 0 0 0</pose>\
    <diffuse>1 1 1 1</diffuse>\
    <specular>0.2 0.2 0.2 1</specular>\
    <attenuation>\
    <range>1000</range>\
    <constant>0.09</constant>\
    <linear>0.001</linear>\
    <quadratic>0.001</quadratic>\
    </attenuation>\
    <direction>-0.5 0.1 -0.9</direction>\
    </light>\
  </sdf>");

  ignition::msgs::Boolean rep;
  bool result;
  bool executed = _node.Request("/world/sim_world/create", create_light, 10000, rep, result);
  std::cout << "Service result: " << executed << " bool result: " << result << std::endl;
}

// Register this plugin
IGNITION_ADD_PLUGIN(LightTuning,
  ignition::gui::Plugin)


#include "LightTuning.moc"