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
#include <std_msgs/msg/float32_multi_array.hpp>
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
        buttons_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("buttons_in", 0);
        axes_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("axes_in", 0);
    }

private:
    void subscriberer_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
    {
        std::vector<int> buttons = msg->buttons;

        auto buttons_message = std_msgs::msg::Int32MultiArray();
        buttons_message.data = buttons;
        std_msgs::msg::MultiArrayDimension buttons_dim;
        buttons_dim.label = "buttons";
        buttons_dim.size = buttons.size();
        buttons_dim.stride = buttons.size();
        buttons_message.layout.dim.push_back(buttons_dim);

        buttons_publisher_->publish(buttons_message);
        

        std::vector<float> axes = msg->axes;

        auto axes_message = std_msgs::msg::Float32MultiArray();
        axes_message.data = axes;
        std_msgs::msg::MultiArrayDimension axes_dim;
        axes_dim.label = "axes";
        axes_dim.size = buttons.size();
        axes_dim.stride = buttons.size();
        buttons_message.layout.dim.push_back(axes_dim);

        axes_publisher_->publish(axes_message);
        
        
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr buttons_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr axes_publisher_;
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
