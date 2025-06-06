//    Copyright 2022 Christoph Hellmann Santos
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#include "canopen_core/lifecycle_manager.hpp"

namespace ros2_canopen
{

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleManager::on_configure(const rclcpp_lifecycle::State & state)
{
  this->get_parameter<std::string>("container_name", this->container_name_);

  bool res = this->load_from_config();
  if (!res)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load from config");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  this->master_reset_client_ = this->create_client<std_srvs::srv::Trigger>(
    "master/reset", rclcpp::QoS(10), cbg_clients
  );

  this->container_init_driver_client_ = 
    this->create_client<canopen_interfaces::srv::CONode>(
      this->container_name_ + "/init_driver", rclcpp::QoS(10), cbg_clients
  );

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleManager::on_activate(const rclcpp_lifecycle::State & state)
{
  if (!this->bring_up_all())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleManager::on_deactivate(const rclcpp_lifecycle::State & state)
{
  if (!this->bring_down_all())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleManager::on_cleanup(const rclcpp_lifecycle::State & state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleManager::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void LifecycleManager::init(std::shared_ptr<ros2_canopen::ConfigurationManager> config)
{
  this->config_ = config;
}

bool LifecycleManager::load_from_config()
{
  std::vector<std::string> devices;
  uint32_t count = this->config_->get_all_devices(devices);
  RCLCPP_INFO(this->get_logger(), "Configuring for %u devices.", count);

  // Find master in configuration
  for (auto it = devices.begin(); it != devices.end(); it++)
  {
    uint8_t node_id = config_->get_entry<uint8_t>(*it, "node_id").value();
    std::string change_state_client_name = *it;
    std::string get_state_client_name = *it;
    get_state_client_name += "/get_state";
    change_state_client_name += "/change_state";
    RCLCPP_INFO(this->get_logger(), "Found %s (node_id=%hu)", it->c_str(), node_id);
    device_names_to_ids.emplace(*it, node_id);
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client =
      this->create_client<lifecycle_msgs::srv::GetState>(
        get_state_client_name, rclcpp::QoS(10), cbg_clients);

    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client =
      this->create_client<lifecycle_msgs::srv::ChangeState>(
        change_state_client_name, rclcpp::QoS(10), cbg_clients);

    this->drivers_get_state_clients.emplace(node_id, get_state_client);
    this->drivers_change_state_clients.emplace(node_id, change_state_client);

    if (it->find("master") != std::string::npos)
    {
      this->master_id_ = node_id;
    }
  }
  return true;
}

unsigned int LifecycleManager::get_state(uint8_t node_id, std::chrono::seconds time_out)
{
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto client = this->drivers_get_state_clients[node_id];
  if (!client->wait_for_service(time_out))
  {
    RCLCPP_ERROR(get_logger(), "Service %s is not available.", client->get_service_name());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }

  // We send the service request for asking the current
  // state of the lc_talker node.
  auto future_result = client->async_send_request(request);

  // Let's wait until we have the answer from the node.
  // If the request times out, we return an unknown state.
  auto future_status = wait_for_result(future_result, time_out);

  if (future_status != std::future_status::ready)
  {
    RCLCPP_ERROR(
      get_logger(), "Server time out while getting current state for node 0x%X", node_id);
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
  auto result = future_result.get()->current_state;
  return result.id;
}

bool LifecycleManager::change_state(
  uint8_t node_id, uint8_t transition, std::chrono::seconds time_out)
{
  auto client = this->drivers_change_state_clients[node_id];
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  if (!client->wait_for_service(time_out))
  {
    RCLCPP_ERROR(get_logger(), "Service %s is not available.", client->get_service_name());
    return false;
  }

  // We send the request with the transition we want to invoke.
  auto future_result = client->async_send_request(request);

  // Let's wait until we have the answer from the node.
  // If the request times out, we return an unknown state.
  auto future_status = wait_for_result(future_result, time_out);

  if (future_status != std::future_status::ready)
  {
    RCLCPP_ERROR(
      get_logger(), "Server time out while getting current state for node 0x%X", node_id);
    return false;
  }

  if (future_result.get()->success)
  {
    return true;
  }
  else
  {
    RCLCPP_WARN(
      get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
    return false;
  }
  return false;
}

bool LifecycleManager::bring_up_master()
{
  auto state = this->get_state(master_id_, 3s);
  if (state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to bring up master. Master not in unconfigured state.");
    return false;
  }
  RCLCPP_DEBUG(this->get_logger(), "Master (node_id=%hu) has state unconfigured.", master_id_);
  if (!this->change_state(master_id_, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, 3s))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to bring up master. Configure Transition failed.");
    return false;
  }
  RCLCPP_DEBUG(this->get_logger(), "Master (node_id=%hu) has state inactive.", master_id_);
  if (!this->change_state(master_id_, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, 3s))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to bring up master. Activate Transition failed.");
    return false;
  }
  RCLCPP_DEBUG(this->get_logger(), "Master (node_id=%hu) has state active.", master_id_);
  return true;
}

bool LifecycleManager::bring_down_master()
{
  this->change_state(master_id_, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, 3s);
  this->change_state(master_id_, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, 3s);

  auto state = this->get_state(master_id_, 3s);

  if (state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    return false;
  }

  return true;
}

bool LifecycleManager::bring_up_driver_configure(std::string device_name)
{
  auto node_id = this->device_names_to_ids[device_name];
  RCLCPP_DEBUG(this->get_logger(), "Configure %s with id %u", device_name.c_str(), node_id);
  auto master_state = this->get_state(master_id_, 3s);
  if (master_state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to bring up %s. Master not in active state.",
      device_name.c_str());
    return false;
  }
  auto state = this->get_state(node_id, 3s);
  if (state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to bring up %s. Not in unconfigured state.", device_name.c_str());
    return false;
  }
  RCLCPP_DEBUG(
    this->get_logger(), "%s (node_id=%hu) has state unconfigured. Attempting to configure.",
    device_name.c_str(), node_id);
  if (!this->change_state(node_id, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, 3s))
  {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to bring up %s. Configure Transition failed.",
      device_name.c_str());
    return false;
  }
  RCLCPP_DEBUG(
    this->get_logger(), "%s (node_id=%hu) has state inactive.",
    device_name.c_str(), node_id);
  return true;
}

bool LifecycleManager::bring_up_driver_activate(std::string device_name)
{
  auto node_id = this->device_names_to_ids[device_name];
  RCLCPP_DEBUG(this->get_logger(), "Activate node %s with id %u", device_name.c_str(), node_id);
  auto master_state = this->get_state(master_id_, 3s);
  if (master_state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to bring up %s. Master not in active state.",
      device_name.c_str());
    return false;
  }
  auto state = this->get_state(node_id, 3s);
  if (state != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to bring up %s. Not in inactive state.", device_name.c_str());
    return false;
  }
  RCLCPP_DEBUG(
    this->get_logger(), "%s (node_id=%hu) has state inactive. Attempting to activate.",
    device_name.c_str(), node_id);
  if (!this->change_state(node_id, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, 3s))
  {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to bring up %s. Activate Transition failed.",
      device_name.c_str());
    return false;
  }
  RCLCPP_DEBUG(
    this->get_logger(), "%s (node_id=%hu) has state active.",
    device_name.c_str(), node_id);
  return true;
}

bool LifecycleManager::bring_down_driver(std::string device_name)
{
  auto node_id = this->device_names_to_ids[device_name];

  this->change_state(node_id, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, 3s);
  this->change_state(node_id, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, 3s);
  auto state = this->get_state(node_id, 3s);
  if (state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    return false;
  }
  return true;
}

bool LifecycleManager::bring_up_all()
{
  RCLCPP_INFO(this->get_logger(), "Bring up master");
  if (!this->bring_up_master())
  {
    return false;
  }

  for (auto it = this->device_names_to_ids.begin(); it != this->device_names_to_ids.end(); ++it)
  {
    if (it->first.find("master") == std::string::npos)
    {
      RCLCPP_INFO(this->get_logger(), "Configure %s", it->first.c_str());
      if (!this->bring_up_driver_configure(it->first))
      {
        return false;
      }
    }
  }

  for (auto it = this->device_names_to_ids.begin(); it != this->device_names_to_ids.end(); ++it)
  {
    if (it->first.find("master") == std::string::npos)
    {
      RCLCPP_INFO(this->get_logger(), "Init driver: %s (nodeid=0x%X)",
          it->first.c_str(), it->second);
      if (!this->container_init_driver(it->second, 3s))
      {
        return false;
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Master reset");
  if (!this->master_reset(3s))
  {
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Activate drivers");
  for (auto it = this->device_names_to_ids.begin(); it != this->device_names_to_ids.end(); ++it)
  {
    if (it->first.find("master") == std::string::npos)
    {
      RCLCPP_INFO(this->get_logger(), "Activate %s", it->first.c_str());
      if (!this->bring_up_driver_activate(it->first))
      {
        return false;
      }
    }
  }
  return true;
}

bool LifecycleManager::bring_down_all()
{
  RCLCPP_INFO(this->get_logger(), "Bring Down all");
  for (auto it = this->device_names_to_ids.begin(); it != this->device_names_to_ids.end(); ++it)
  {
    if (it->first.compare("master") != 0)
    {
      if (!this->bring_down_driver(it->first))
      {
        return false;
      }
    }
  }
  if (!this->bring_down_master())
  {
    return false;
  }

  return true;
}

bool LifecycleManager::container_init_driver(uint8_t node_id, std::chrono::seconds time_out)
{
  auto request = std::make_shared<canopen_interfaces::srv::CONode::Request>();
  request->nodeid = node_id;

  if (!this->container_init_driver_client_->wait_for_service(time_out))
  {
    RCLCPP_ERROR(get_logger(), "Service %s is not available.",
        this->container_init_driver_client_->get_service_name());
    return false;
  }

  auto future_result =
      this->container_init_driver_client_->async_send_request(request);

  auto future_status = wait_for_result(future_result, time_out);

  if (future_status != std::future_status::ready)
  {
    RCLCPP_ERROR(get_logger(), "Server time out while calling %s (nodeid=0x%X)",
        this->container_init_driver_client_->get_service_name(),
        node_id);
    return false;
  }

  if (future_result.get()->success)
  {
    return true;
  }

  RCLCPP_WARN(get_logger(),
      "Failed to init_driver for node 0x%X", node_id);
  return false;
}

bool LifecycleManager::master_reset(std::chrono::seconds time_out)
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  if (!this->master_reset_client_->wait_for_service(time_out))
  {
    RCLCPP_ERROR(get_logger(), "Service %s is not available.",
        this->master_reset_client_->get_service_name());
    return false;
  }

  auto future_result = this->master_reset_client_->async_send_request(request);

  auto future_status = wait_for_result(future_result, time_out);

  if (future_status != std::future_status::ready)
  {
    RCLCPP_ERROR(get_logger(), "Server time out while calling %s",
        this->master_reset_client_->get_service_name());
    return false;
  }

  if (future_result.get()->success)
  {
    return true;
  }

  RCLCPP_WARN(get_logger(), "Failed to call %s",
      this->master_reset_client_->get_service_name());
  return false;
}

}  // namespace ros2_canopen

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::LifecycleManager)
