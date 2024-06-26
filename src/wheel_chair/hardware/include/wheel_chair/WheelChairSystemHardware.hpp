// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once


#include <iostream>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using namespace hardware_interface;

namespace wheel_chair
{

  class WheelChairSystemHardware : public hardware_interface::SystemInterface
  {
    public:
      RCLCPP_SHARED_PTR_DEFINITIONS(WheelChairSystemHardware)

      CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

      CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
        
        
      CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

      hardware_interface::return_type read() override;

      hardware_interface::return_type write() override;

    private:
      // Parameters for the wheel_chair simulation
      // double hw_start_sec_;
      // double hw_stop_sec_;

      // port, baudrate, timeout in milliseconds
      std::shared_ptr<serial::Serial> base_port;


      // Store the command for the simulated robot
      std::vector<double> hw_commands_;
      std::vector<double> hw_positions_;
      std::vector<double> hw_velocities_;
  };

}  // namespace wheel_chair
