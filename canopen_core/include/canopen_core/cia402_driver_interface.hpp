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

#ifndef CANOPEN_402_DRIVER__402_DRIVER_INTERFACE_HPP_
#define CANOPEN_402_DRIVER__402_DRIVER_INTERFACE_HPP_

#include <functional>
#include <stdint.h>
#include <lely/coapp/node.hpp>
#include "canopen_core/exchange.hpp"

namespace ros2_canopen
{
/**
 * @brief Abstract Class for a CANopen 402 Device Driver
 */

class Cia402DriverInterface
{
public:
  virtual bool reset_node_nmt_command() = 0;

  virtual bool start_node_nmt_command() = 0;

  virtual bool tpdo_transmit(ros2_canopen::COData & data) = 0;

  virtual bool sdo_write(ros2_canopen::COData & data) = 0;

  virtual bool sdo_read(ros2_canopen::COData & data) = 0;

  virtual void register_nmt_state_cb(std::function<void(lely::canopen::NmtState, uint8_t)> nmt_state_cb) = 0;

  virtual void register_rpdo_cb(std::function<void(COData, uint8_t)> rpdo_cb) = 0;

  virtual double get_speed() = 0;

  virtual double get_speed_avg() = 0;

  virtual double get_position() = 0;

  virtual double get_current() = 0;

  virtual bool set_target(double target) = 0;

  virtual bool init_motor() = 0;

  virtual bool recover_motor() = 0;

  virtual bool halt_motor() = 0;

  virtual uint16_t get_mode() = 0;

  virtual bool set_operation_mode(uint16_t mode) = 0;
};
}  // namespace ros2_canopen

#endif
