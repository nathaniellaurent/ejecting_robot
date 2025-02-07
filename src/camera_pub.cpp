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
#include <thread>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
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

        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image" + std::to_string(camera_index), 0);
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 0);

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

        std::cout << "objpoints size: " << objpoints.size() << std::endl;

        if (objpoints.size() < 40)
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
    void detect_pose(cv::Mat frame)
    {
        cv::Mat undistorted_frame;
        cv::undistort(frame, undistorted_frame, cameraMatrix, distCoeffs);

        if (undistorted_frame.cols < 640 || undistorted_frame.rows < 480)
        {
            cv::resize(undistorted_frame, undistorted_frame, cv::Size(), 2.0, 2.0);
        }

        // vector to store the pixel coordinates of detected checker board corners

        // Looping over all the images in the directory

        // Detect ArUco markers
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);

        cv::Mat objPoints(4, 1, CV_32FC3);
        objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-marker_length / 2.f, marker_length / 2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(marker_length / 2.f, marker_length / 2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(marker_length / 2.f, -marker_length / 2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-marker_length / 2.f, -marker_length / 2.f, 0);
        // Draw detected markers

        size_t nMarkers = markerCorners.size();
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

        if (!markerIds.empty())
        {
            // Calculate pose for each marker
            for (size_t i = 0; i < nMarkers; i++)
            {
                solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
            }

            cv::Mat rotation_matrix;
            cv::Vec3d new_rvec;
            new_rvec[0] = rvecs.at(0)[2];
            new_rvec[1] = rvecs.at(0)[0];
            new_rvec[2] = rvecs.at(0)[1];

            cv::Vec3d new_tvec;
            new_tvec[0] = tvecs.at(0)[2];
            new_tvec[1] = tvecs.at(0)[0];
            new_tvec[2] = tvecs.at(0)[1];

            cv::Rodrigues(new_rvec, rotation_matrix);
            cv::Mat inverted_rotation_matrix = rotation_matrix.t();

            cv::Mat inverted_translation_mat = -inverted_rotation_matrix * cv::Mat(new_tvec);
            cv::Vec3d inverted_translation_vector;
            inverted_translation_vector[0] = inverted_translation_mat.at<double>(0);
            inverted_translation_vector[1] = inverted_translation_mat.at<double>(1);
            inverted_translation_vector[2] = inverted_translation_mat.at<double>(2);

            Eigen::Matrix3f eigen_inverted_rotation;
            eigen_inverted_rotation(0, 0) = inverted_rotation_matrix.at<double>(0, 0);
            eigen_inverted_rotation(0, 1) = inverted_rotation_matrix.at<double>(0, 1);
            eigen_inverted_rotation(0, 2) = inverted_rotation_matrix.at<double>(0, 2);
            eigen_inverted_rotation(1, 0) = inverted_rotation_matrix.at<double>(1, 0);
            eigen_inverted_rotation(1, 1) = inverted_rotation_matrix.at<double>(1, 1);
            eigen_inverted_rotation(1, 2) = inverted_rotation_matrix.at<double>(1, 2);
            eigen_inverted_rotation(2, 0) = inverted_rotation_matrix.at<double>(2, 0);
            eigen_inverted_rotation(2, 1) = inverted_rotation_matrix.at<double>(2, 1);
            eigen_inverted_rotation(2, 2) = inverted_rotation_matrix.at<double>(2, 2);

            Eigen::Quaternionf inverted_quaternion(eigen_inverted_rotation);

            inverted_quaternion.normalize();

            geometry_msgs::msg::PoseStamped pose_message;

            pose_message.header.stamp = this->now();
            pose_message.header.frame_id = "world";

            pose_message.pose.position.x = inverted_translation_vector[0];
            pose_message.pose.position.y = inverted_translation_vector[1];
            pose_message.pose.position.z = inverted_translation_vector[2];

            pose_message.pose.orientation.x = inverted_quaternion.x();
            pose_message.pose.orientation.y = inverted_quaternion.y();
            pose_message.pose.orientation.z = inverted_quaternion.z();
            pose_message.pose.orientation.w = inverted_quaternion.w();

            for (size_t i = 0; i < markerIds.size(); i++)
            {
                RCLCPP_INFO(this->get_logger(), "Marker ID: %d", markerIds[i]);
                RCLCPP_INFO(this->get_logger(), "Translation vector: [%f, %f, %f]", inverted_translation_vector[0], inverted_translation_vector[1], inverted_translation_vector[2]);
                RCLCPP_INFO(this->get_logger(), "Rotation vector: [%f, %f, %f]", rvecs[i][0], rvecs[i][1], rvecs[i][2]);
                RCLCPP_INFO(this->get_logger(), "Quaternion: [%f, %f, %f, %f]", inverted_quaternion.x(), inverted_quaternion.y(), inverted_quaternion.z(), inverted_quaternion.w());
            }

            pose_publisher_->publish(pose_message);
        }

        if (!markerIds.empty())
        {
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "No Tags Detected");
        }
        // Output the positions of the markers

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

        image_publisher_->publish(image_msg);

        cv::Mat resized_frame;
        cv::resize(frame, resized_frame, cv::Size(), 0.7, 0.7);
        cv::imshow("ID: " + std::to_string(camera_index), frame);
        cv::waitKey(1);
    }

    void getFrame(cv::Mat &frame, std::shared_ptr<cv::VideoCapture> cap)
    {
        try
        {
            cap->read(frame);
        }
        catch (cv::Exception e)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Error reading frame");
        }
    }
    void getFrameWrapper(cv::Mat &frame, std::shared_ptr<cv::VideoCapture> cap)
    {
        std::mutex m;
        std::condition_variable cv;

        std::thread t([&cv, &frame, &cap, this]()
                      {
                          getFrame(frame, cap);
                          cv.notify_one(); });

        t.detach();

        {
            std::unique_lock<std::mutex> l(m);
            if (cv.wait_for(l, 1s) == std::cv_status::timeout)
                throw std::runtime_error("Timeout");
        }
    }

    void timer_callback()
    {
        cv::Mat frame;

        if (!cap.get()->isOpened())
        {
            try
            {
                std::cout << "Connecting to camera " << camera_index << std::endl;
                cap.get()->open(rtsp_url, cv::CAP_FFMPEG);

                std::cout << "Connected successfully to camera " << camera_index << std::endl;
            }
            catch (std::exception e)
            {

                RCLCPP_INFO_STREAM(this->get_logger(), "Camera " << camera_index << " failed to connect. Trying Again");
                rclcpp::sleep_for(1s);
            }
        }

        else if (!camFailed)
        {

            try
            {
                getFrameWrapper(frame, cap);
                detect_pose(frame);
            }
            catch (std::runtime_error &e)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "Camera " << camera_index << " failed to get frame. Reconnecting");
                // cap->release();
                RCLCPP_INFO_STREAM(this->get_logger(), "Camera " << camera_index << "opened status: " << cap->isOpened());

                camFailed = true;
            }
        }
        // try
        // {
        //     getFrameWrapper(frame, cap);
        //     detect_pose(frame);
        // }
        // catch (std::runtime_error &e)
        // {
        //     RCLCPP_INFO_STREAM(this->get_logger(), "Camera " << camera_index << " failed to get frame. Reconnecting");
        //     // cap->release();
        //     RCLCPP_INFO_STREAM(this->get_logger(), "Camera " << camera_index << "opened status: " << cap->isOpened());
        // }
    }

    std::vector<cv::Point2f> corner_pts;
    bool calibrated = false;

    std::vector<std::vector<cv::Point3f>> objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f>> imgpoints;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    size_t count_;

    bool camFailed = false;

    std::shared_ptr<cv::VideoCapture> cap;
    std::string rtsp_url;
    int camera_index;
    float marker_length = 0.15875;
    int CHECKERBOARD[2]{6, 9};
    // cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 434.4572880719159, 0, 618.6044132887367,
    //                         0, 435.2917648761863, 361.3815453027511,
    //                         0, 0, 1);

    // cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0.2116337614583763, -0.2336134487696931, 0.002105638057892645, -0.0008786200891493874, 0.05910128608762473);

    // cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 110.2894455498797, 0, 164.3783581439072,
    //                         0, 113.1385298003699, 124.4976515535277,
    //                         0, 0, 1);

    // cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << .1668463346620745, -0.1694508476023472, -0.0006804338570774129, 4.956087034116383e-05, 0.03606227345997835);
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 86.5543971578997, 0, 151.4114327956759,
                            0, 89.22489153429152, 124.9870606734323,
                            0, 0, 1);

    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0.01440877775584453, -0.02739560808511739, -0.001982290010110622, 0.002271691744549752, 0.003002276168833601);
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::vector<cv::Point2f> corner_pts;
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
