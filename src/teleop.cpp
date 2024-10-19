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
#include <std_msgs/msg/int32.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class TeleopPublisher : public rclcpp::Node
{
public:
    TeleopPublisher()
        : Node("minimal_publisher"), count_(0)
    {


        publisher_ = this->create_publisher<std_msgs::msg::Int32>("in_topic", 0);
        timer_ = this->create_wall_timer(
            1ms, std::bind(&TeleopPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        count_++;
        auto message = std_msgs::msg::Int32();
        message.data = count_;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
        publisher_->publish(message);
        
        
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    size_t count_;

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
