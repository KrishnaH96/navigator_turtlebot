/**
 * @file navigator_turtlebot.cpp
 * @brief ROS 2 node for navigation based on laser scan data
 *
 * @author Krishna Rajesh Hundekari
 * @date 2023-11-29
 * @copyright 2023 Krishna Rajesh Hundekari
 * Apache License Version 2.0, January 2004
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with this
 * work for additional information regarding copyright ownership. The ASF
 * licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

using LaserScanMsg = sensor_msgs::msg::LaserScan;
using TwistMsg = geometry_msgs::msg::Twist;

/**
 * @brief ROS 2 node for navigation based on laser scan data.
 */
class navigator : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new navigator object.
   */
  navigator() : Node("navigator") {
    auto timecallback =
        std::bind(&navigator::laserDataCallback, this, std::placeholders::_1);
    laserDataSub =
        this->create_subscription<LaserScanMsg>("scan", 10, timecallback);
    velocityPublisher = this->create_publisher<TwistMsg>("cmd_vel", 10);
  }

 private:
  /**
   * @brief Callback function for laser scan data.
   * @param scanData The received laser scan data.
   */
  void laserDataCallback(const LaserScanMsg& scanData) {
    if (scanData.header.stamp.sec == 0) {
      return;
    }

    auto laserScanData = scanData.ranges;
    constexpr int startAngle = 330;
    constexpr int endAngle = startAngle + 60;

    for (int i = startAngle; i < endAngle; i++) {
      if (laserScanData[i % 360] < 0.8) {
        moveRobot(0.0, 0.1);
      } else {
        moveRobot(0.1, 0.0);
      }
    }
  }

  /**
   * @brief Move the robot based on specified linear and angular velocities.
   * @param xVelocity The linear velocity.
   * @param zVelocity The angular velocity.
   */
  void moveRobot(float xVelocity, float zVelocity) {
    auto velocityMsg = TwistMsg();
    velocityMsg.linear.x = xVelocity;
    velocityMsg.angular.z = -zVelocity;
    velocityPublisher->publish(velocityMsg);
  }

  // Private member variables
  rclcpp::Subscription<LaserScanMsg>::SharedPtr
      laserDataSub;  ///< Subscription for laser scan data.
  rclcpp::Publisher<TwistMsg>::SharedPtr
      velocityPublisher;  ///< Publisher for robot velocity commands.
};

/**
 * @brief Main function to initialize and run the ROS 2 node.
 * @param argc Number of command-line arguments.
 * @param argv Command-line arguments.
 * @return 0 on successful execution.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigator>());
  rclcpp::shutdown();
  return 0;
}
