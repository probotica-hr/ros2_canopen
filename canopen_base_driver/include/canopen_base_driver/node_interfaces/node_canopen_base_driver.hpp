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

#ifndef NODE_CANOPEN_BASE_DRIVER
#define NODE_CANOPEN_BASE_DRIVER

#include "canopen_base_driver/diagnostic_collector.hpp"
#include "canopen_base_driver/lely_driver_bridge.hpp"
#include "canopen_core/node_interfaces/node_canopen_driver.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "canopen_interfaces/msg/co_data.hpp"
#include "canopen_interfaces/srv/co_read.hpp"
#include "canopen_interfaces/srv/co_write.hpp"

namespace ros2_canopen
{
namespace node_interfaces
{
template <class NODETYPE>
class NodeCanopenBaseDriver : public NodeCanopenDriver<NODETYPE>
{
  static_assert(
    std::is_base_of<rclcpp::Node, NODETYPE>::value ||
      std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
    "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");

protected:
  std::thread nmt_state_publisher_thread_;
  std::thread rpdo_publisher_thread_;
  std::thread emcy_publisher_thread_;
  std::mutex driver_mutex_;
  std::shared_ptr<ros2_canopen::LelyDriverBridge> lely_driver_;
  uint32_t period_ms_;
  int sdo_timeout_ms_;
  bool polling_;

  // nmt state callback
  std::function<void(canopen::NmtState, uint8_t)> nmt_state_cb_;
  // rpdo callback
  std::function<void(COData, uint8_t)> rpdo_cb_;
  // emcy callback
  std::function<void(COEmcy, uint8_t)> emcy_cb_;

  std::shared_ptr<ros2_canopen::SafeQueue<ros2_canopen::COEmcy>> emcy_queue_;
  std::shared_ptr<ros2_canopen::SafeQueue<ros2_canopen::COData>> rpdo_queue_;
  rclcpp::TimerBase::SharedPtr poll_timer_;

  // Diagnostic components
  std::atomic<bool> diagnostic_enabled_;
  uint32_t diagnostic_period_ms_;
  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  std::shared_ptr<DiagnosticsCollector> diagnostic_collector_;

  virtual void poll_timer_callback();
  void nmt_listener();
  virtual void on_nmt(canopen::NmtState nmt_state);
  void rdpo_listener();
  virtual void on_rpdo(COData data);
  void emcy_listener();
  virtual void on_emcy(COEmcy emcy);
  virtual void diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper & stat);

public:
  NodeCanopenBaseDriver(NODETYPE * node);

  virtual ~NodeCanopenBaseDriver()
  {
    if (nmt_state_publisher_thread_.joinable())
    {
      nmt_state_publisher_thread_.join();
    }
    if (rpdo_publisher_thread_.joinable())
    {
      rpdo_publisher_thread_.join();
    }
  }

  virtual void set_master(
    std::shared_ptr<lely::ev::Executor> exec, std::shared_ptr<lely::canopen::AsyncMaster> master) override;

  virtual void init(bool called_from_base);

  virtual void configure(bool called_from_base);

  virtual void activate(bool called_from_base);

  virtual void deactivate(bool called_from_base);

  virtual void cleanup(bool called_from_base);

  virtual void shutdown(bool called_from_base);

  virtual void add_to_master();

  virtual void remove_from_master();

  /**
   * @brief Register a callback for NMT state change
   *
   * @param nmt_state_cb
   */
  void register_nmt_state_cb(std::function<void(canopen::NmtState, uint8_t)> nmt_state_cb)
  {
    nmt_state_cb_ = std::move(nmt_state_cb);
  }

  /**
   * @brief Register a callback for RPDO
   *
   * @param rpdo_cb
   */
  void register_rpdo_cb(std::function<void(COData, uint8_t)> rpdo_cb)
  {
    rpdo_cb_ = std::move(rpdo_cb);
  }

  /**
   * @brief Register a callback for EMCY
   *
   * @param emcy_cb
   */
  void register_emcy_cb(std::function<void(COEmcy, uint8_t)> emcy_cb)
  {
    emcy_cb_ = std::move(emcy_cb);
  }
};
typedef NodeCanopenBaseDriver<rclcpp::Node> NCBDNode;
typedef NodeCanopenBaseDriver<rclcpp_lifecycle::LifecycleNode> NCBDLifecycleNode;
}  // namespace node_interfaces
}  // namespace ros2_canopen

#endif
