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
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher()
        : Node("minimal_publisher"), count_(0)
    {

        for (int i{0}; i < CHECKERBOARD[1]; i++)
        {
            for (int j{0}; j < CHECKERBOARD[0]; j++)
                objp.push_back(cv::Point3f(j, i, 0));
        }

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
    void calibrate_camera(cv::Mat frame)
    {
        cv::Mat calibrateCameraMatrix, calibrateDistCoeffs, R, T;

        cv::Mat gray, original;

        original = frame.clone();
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true
        if (cv::waitKey(1) == 'c')
        {
            bool successInside = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

            /*
             * If desired number of corner are detected,
             * we refine the pixel coordinates and display
             * them on the images of checker board
             */
            if (successInside)
            {
                cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

                // refining pixel coordinates for given 2d points.
                cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

                // Displaying the detected corner points on the checker board
                cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, successInside);

                objpoints.push_back(objp);
                imgpoints.push_back(corner_pts);
                cv::Mat resized_frame;
                cv::resize(frame, resized_frame, cv::Size(), 0.7, 0.7);
                cv::imshow("Corners", resized_frame);
                cv::waitKey(1);
            }
        }

        cv::Mat resized_frame;
        cv::resize(original, resized_frame, cv::Size(), 0.7, 0.7);
        cv::imshow("Image", resized_frame);
        cv::waitKey(1);

        if (objpoints.size() < 30)
        {
            RCLCPP_INFO(this->get_logger(), "Not enough points for calibration");
            return;
        }
        else if (calibrated)
        {
            cv::Mat undistorted_frame;
            cv::undistort(frame, undistorted_frame, calibrateCameraMatrix, calibrateDistCoeffs);

            if (undistorted_frame.cols < 640 || undistorted_frame.rows < 480)
            {
                cv::resize(undistorted_frame, undistorted_frame, cv::Size(), 2.0, 2.0);
            }
            cv::Mat resized_frame;
            cv::resize(undistorted_frame, resized_frame, cv::Size(), 0.7, 0.7);
            cv::imshow("Undistorted ID: " + std::to_string(camera_index), resized_frame);
            cv::waitKey(1);
        }
        else
        {

            /*
             * Performing camera calibration by
             * passing the value of known 3D points (objpoints)
             * and corresponding pixel coordinates of the
             * detected corners (imgpoints)
             */
            cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), calibrateCameraMatrix, calibrateDistCoeffs, R, T);

            std::cout << "cameraMatrix : " << calibrateCameraMatrix << std::endl;
            std::cout << "distCoeffs : " << calibrateDistCoeffs << std::endl;
            std::cout << "Rotation vector : " << R << std::endl;
            std::cout << "Translation vector : " << T << std::endl;
        }
            
    }

    void timer_callback()
    {
        cv::Mat frame;
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
                cap.get()->open(rtsp_url, 0, params);

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

            cap.get()->read(frame);

            
            cv::Mat undistorted_frame;
            cv::undistort(frame, undistorted_frame, cameraMatrix, distCoeffs);

            if (undistorted_frame.cols < 640 || undistorted_frame.rows < 480)
            {
                cv::resize(undistorted_frame, undistorted_frame, cv::Size(), 2.0, 2.0);
            }
            cv::Mat resized_frame;
            cv::resize(undistorted_frame, resized_frame, cv::Size(), 0.7, 0.7);
            cv::imshow("Undistorted ID: " + std::to_string(camera_index), resized_frame);
            cv::waitKey(1);
            

            // vector to store the pixel coordinates of detected checker board corners

            // Looping over all the images in the directory
        }

        // Detect ArUco markers
        // cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
        // std::vector<int> markerIds;
        // std::vector<std::vector<cv::Point2f>> markerCorners;
        // cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);

        // cv::Mat objPoints(4, 1, CV_32FC3);
        // objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-marker_length / 2.f, marker_length / 2.f, 0);
        // objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(marker_length / 2.f, marker_length / 2.f, 0);
        // objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(marker_length / 2.f, -marker_length / 2.f, 0);
        // objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-marker_length / 2.f, -marker_length / 2.f, 0);
        // // Draw detected markers

        // size_t nMarkers = markerCorners.size();
        // std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

        // if (!markerIds.empty())
        // {
        //     // Calculate pose for each marker
        //     for (size_t i = 0; i < nMarkers; i++)
        //     {
        //         solvePnP(objPoints, markerCorners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
        //     }
        // }

        // if (!markerIds.empty())
        // {
        //     cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        // }
        // else
        // {
        //     RCLCPP_INFO_STREAM(this->get_logger(), "No Tags Detected");

        // Undistort the image if calibrated

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

    std::vector<cv::Point2f> corner_pts;
    bool calibrated = false;

    std::vector<std::vector<cv::Point3f>> objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f>> imgpoints;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    size_t count_;

    std::shared_ptr<cv::VideoCapture> cap;
    std::string rtsp_url;
    int camera_index;
    float marker_length = 5;
    int CHECKERBOARD[2]{6, 9};
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 434.4572880719159, 0, 618.6044132887367,
                            0, 435.2917648761863, 361.3815453027511,
                            0, 0, 1);

    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0.2116337614583763, -0.2336134487696931, 0.002105638057892645, -0.0008786200891493874, 0.05910128608762473);
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::vector<cv::Point2f> corner_pts;
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
