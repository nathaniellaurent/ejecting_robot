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
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
        : Node("minimal_publisher"), count_(0)
    {
        

    
        this->declare_parameter("camera_index", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("rstp_url", rclcpp::PARAMETER_STRING);
        
        

        rclcpp::Parameter camera_param = this->get_parameter("camera_index");
        camera_index = camera_param.as_int();

        rclcpp::Parameter rstp_url_param = this->get_parameter("rstp_url");
        rtsp_url = rstp_url_param.as_string();

        cv::VideoCapture vidCap(rtsp_url);
        cap = std::make_shared<cv::VideoCapture>(vidCap);

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image" + std::to_string(camera_index), 0);
        timer_ = this->create_wall_timer(
            1ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        
        
        cap.get()->read(frame);
        cv::imshow("ID: " + std::to_string(camera_index), frame);
        cv::waitKey(1);

        

        auto message = std_msgs::msg::String();
        message.data = "ID: " + std::to_string(camera_index) + " Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

        cv_bridge::CvImage cvImage;
        cvImage.header.stamp = this->now();
        cvImage.image = frame;
        
        publisher_->publish(*cvImage.toImageMsg());
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    size_t count_;

    std::shared_ptr<cv::VideoCapture> cap;
    std::string rtsp_url;
    int camera_index;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
