#ifndef SRC__RMF_PLUGINS__UTILS_HPP
#define SRC__RMF_PLUGINS__UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <sdf/Element.hh>
#include <memory>

namespace rmf_plugins_utils {

/////////////////////////////////////////////////////////////////////////////////////////////
enum Simulator {Ignition, Gazebo};

// Holds an identifier referring to either an Ignition or Gazebo classic entity
// Contains either a uint64_t value or a std::string value depending on `sim_type`
// Enables functions to be written that generically operate on both Ignition or Gazebo entities
struct SimEntity
{
  Simulator sim_type;
  uint64_t entity; // If used for Ignition Gazebo
  std::string name; // If used for Gazebo classic

  SimEntity(uint64_t en)
  : sim_type(Ignition), entity(en)
  {
    name = "";
  }
  SimEntity(std::string nm)
  : sim_type(Gazebo), name(nm)
  {
    entity = 0;
  }

  const std::string& get_name() const
  {
    if (sim_type != Gazebo)
    {
      std::cerr << "SimEntity Ignition object does not hold a name."
                << std::endl;
    }
    return name;
  }

  uint64_t get_entity() const
  {
    if (sim_type != Ignition)
    {
      std::cerr << "SimEntity Gazebo object does not hold a uint64_t entity."
                << std::endl;
    }
    return entity;
  }
};

/////////////////////////////////////////////////////////////////////////////////////////////////
// TODO(MXG): Refactor the use of this function to replace it with
// compute_desired_rate_of_change()
double compute_ds(
  double s_target,
  double v_actual,
  double v_max,
  double accel_nom,
  double accel_max,
  double dt);

struct MotionParams
{
  double v_max = 0.2;
  double a_max = 0.1;
  double a_nom = 0.08;
  double dx_min = 0.01;
  double f_max = 10000000.0;
};

double compute_desired_rate_of_change(
  double _s_target,
  double _v_actual,
  const MotionParams& _motion_params,
  const double _dt);

bool get_element_required(
  const sdf::ElementPtr& _sdf,
  const std::string& _element_name,
  sdf::ElementPtr& _element);

rclcpp::Time simulation_now(double t);

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T>
bool get_sdf_attribute_required(const sdf::ElementPtr& sdf,
  const std::string& attribute_name, T& value)
{
  if (sdf->HasAttribute(attribute_name))
  {
    if (sdf->GetAttribute(attribute_name)->Get(value))
    {
      std::cout << "Using specified attribute value [" << value
                << "] for property [" << attribute_name << "]"
                << std::endl;
      return true;
    }
    else
    {
      std::cerr << "Failed to parse sdf attribute for [" << attribute_name
                << "]" << std::endl;
    }
  }
  else
  {
    std::cerr << "Attribute [" << attribute_name << "] not found" << std::endl;
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T>
bool get_sdf_param_required(const sdf::ElementPtr& sdf,
  const std::string& parameter_name, T& value)
{
  if (sdf->HasElement(parameter_name))
  {
    if (sdf->GetElement(parameter_name)->GetValue()->Get(value))
    {
      std::cout << "Using specified value [" << value << "] for property ["
                << parameter_name << "]" << std::endl;
      return true;
    }
    else
    {
      std::cerr << "Failed to parse sdf value for [" << parameter_name << "]"
                <<std::endl;
    }
  }
  else
  {
    std::cerr << "Property [" << parameter_name << "] not found" << std::endl;
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T>
void get_sdf_param_if_available(const sdf::ElementPtr& sdf,
  const std::string& parameter_name, T& value)
{
  if (sdf->HasElement(parameter_name))
  {
    if (sdf->GetElement(parameter_name)->GetValue()->Get(value))
    {
      std::cout << "Using specified value [" << value << "] for property ["
                << parameter_name << "]" << std::endl;
    }
    else
    {
      std::cerr << "Failed to parse sdf value for [" << parameter_name
                << "]" << std::endl;
    }
  }
  else
  {
    std::cout << "Using default value [" << value << "] for property ["
              << parameter_name << "]" << std::endl;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename ResultMsgT>
std::shared_ptr<ResultMsgT> make_response(uint8_t status,
  const double sim_time,
  const std::string& request_guid,
  const std::string& guid)
{
  std::shared_ptr<ResultMsgT> response = std::make_shared<ResultMsgT>();
  response->time = simulation_now(sim_time);
  response->request_guid = request_guid;
  response->source_guid = guid;
  response->status = status;
  return response;
}

} // namespace rmf_plugins_utils

#endif // SRC__RMF_PLUGINS__UTILS_HPP
