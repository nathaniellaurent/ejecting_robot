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
#include <opencv2/videoio.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher()
        : Node("minimal_publisher"), count_(0)
    {

        this->declare_parameter("camera_index", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("rstp_url", rclcpp::PARAMETER_STRING);

        rclcpp::Parameter camera_param = this->get_parameter("camera_index");
        camera_index = camera_param.as_int();

        rclcpp::Parameter rstp_url_param = this->get_parameter("rstp_url");
        rtsp_url = rstp_url_param.as_string();

        cap = std::make_shared<cv::VideoCapture>();

        cap.get()->setExceptionMode(true);

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image" + std::to_string(camera_index), 0);
        timer_ = this->create_wall_timer(
            1ms, std::bind(&CameraPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        std::vector<int> set_params;
        set_params.push_back(cv::CAP_PROP_OPEN_TIMEOUT_MSEC);
        set_params.push_back(1000);

        const std::vector<int> params = set_params;

        bool success = true;
        if (!cap.get()->isOpened())
        {
            try
            {
                std::cout << "Connecting to camera " << camera_index << std::endl;
                cap.get()->open(rtsp_url,0, params);

                std::cout << "Connected successfully to camera " << camera_index << std::endl;
                success = true;
            }
            catch (std::exception e)
            {

                RCLCPP_INFO_STREAM(this->get_logger(), "Camera " << camera_index << " failed to connect. Trying Again");
                rclcpp::sleep_for(1s);
                success = false;
            }
        }
        if (success)
        {
            cv::Mat before_frame;
            cv::Mat frame;

            cap.get()->read(frame);
            cv::flip(frame, before_frame, 0);
            cv::flip(before_frame, frame, 1);
            cv::imshow("ID: " + std::to_string(camera_index), frame);
            cv::waitKey(1);

            if (count_ % 100 == 0)
            {

                auto message = std_msgs::msg::String();
                message.data = "ID: " + std::to_string(camera_index) + " Count: " + std::to_string(count_);
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            }
            count_++;

            cv_bridge::CvImage cvImage;
            cvImage.header.stamp = this->now();
            cvImage.image = frame;
            sensor_msgs::msg::Image image_msg = *cvImage.toImageMsg();
            image_msg.set__encoding("bgr8");

            publisher_->publish(image_msg);
        }
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
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
