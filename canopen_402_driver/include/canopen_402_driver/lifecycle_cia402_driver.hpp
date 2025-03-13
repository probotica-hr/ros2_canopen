//    Copyright 2023 Christoph Hellmann Santos
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//

#ifndef CANOPEN_402_DRIVER__CANOPEN_LIFECYCLE_402_DRIVER_HPP_
#define CANOPEN_402_DRIVER__CANOPEN_LIFECYCLE_402_DRIVER_HPP_

#include "canopen_402_driver/node_interfaces/node_canopen_402_driver.hpp"
#include "canopen_core/driver_node.hpp"
#include "canopen_core/cia402_driver_interface.hpp"

namespace ros2_canopen
{

class LifecycleCia402DriverBase : public ros2_canopen::LifecycleCanopenDriver, public ros2_canopen::Cia402DriverInterface
{
public:
  LifecycleCia402DriverBase(rclcpp::NodeOptions node_options)
  : LifecycleCanopenDriver(node_options) { };
};

/**
 * @brief Lifecycle 402 Driver
 *
 * A very basic driver without any functionality.
 *
 */
class LifecycleCia402Driver : public LifecycleCia402DriverBase
{
  std::shared_ptr<node_interfaces::NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>>
    node_canopen_402_driver_;

public:
  LifecycleCia402Driver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());

  virtual bool reset_node_nmt_command() override
  {
    return node_canopen_402_driver_->reset_node_nmt_command();
  }

  virtual bool start_node_nmt_command() override
  {
    return node_canopen_402_driver_->start_node_nmt_command();
  }

  virtual bool tpdo_transmit(ros2_canopen::COData & data) override
  {
    return node_canopen_402_driver_->tpdo_transmit(data);
  }

  virtual bool sdo_write(ros2_canopen::COData & data) override
  {
    return node_canopen_402_driver_->sdo_write(data);
  }

  virtual bool sdo_read(ros2_canopen::COData & data) override
  {
    return node_canopen_402_driver_->sdo_read(data);
  }

  virtual void register_nmt_state_cb(std::function<void(canopen::NmtState, uint8_t)> nmt_state_cb) override
  {
    node_canopen_402_driver_->register_nmt_state_cb(nmt_state_cb);
  }

  virtual void register_rpdo_cb(std::function<void(COData, uint8_t)> rpdo_cb) override
  {
    node_canopen_402_driver_->register_rpdo_cb(rpdo_cb);
  }

  virtual double get_speed() override { return node_canopen_402_driver_->get_speed(); }

  virtual double get_position() override { return node_canopen_402_driver_->get_position(); }

  virtual bool set_target(double target) override { return node_canopen_402_driver_->set_target(target); }

  virtual bool init_motor() override { return node_canopen_402_driver_->init_motor(); }

  virtual bool recover_motor() override { return node_canopen_402_driver_->recover_motor(); }

  virtual bool halt_motor() override { return node_canopen_402_driver_->halt_motor(); }

  virtual uint16_t get_mode() override { return node_canopen_402_driver_->get_mode(); }

  virtual bool set_operation_mode(uint16_t mode) override
  {
    return node_canopen_402_driver_->set_operation_mode(mode);
  }
};
}  // namespace ros2_canopen

#endif  // CANOPEN_402_DRIVER__CANOPEN_402_DRIVER_HPP_
