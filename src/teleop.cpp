// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int32.hpp>


using std::placeholders::_1;

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class TeleopPublisher : public rclcpp::Node
{
public:
    TeleopPublisher()
        : Node("minimal_publisher")
    {

        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 1, std::bind(&TeleopPublisher::subscriberer_callback, this, _1));
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("in_topic", 0);
    }

private:
    void subscriberer_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
    {
        std::vector<int> buttons = msg->buttons;

        auto message = std_msgs::msg::Int32MultiArray();
        message.data = buttons;
        std_msgs::msg::MultiArrayDimension dim;
        dim.label = "buttons";
        dim.size = buttons.size();
        dim.stride = buttons.size();
        message.layout.dim.push_back(dim);

        publisher_->publish(message);
        
        
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

    std::string rtsp_url;
    int camera_index;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopPublisher>());
    rclcpp::shutdown();
    return 0;
}
