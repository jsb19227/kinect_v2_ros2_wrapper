/*
* Kinect2 ROS Driver
*/


#ifndef KINECT2_DRIVER_HPP_
#define KINECT2_DRIVER_HPP_

//ROS Includes

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

//Libfreenect2 Includes
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

//STD Includes
#include <string.h>
#include <stdexcept>
#include <vector>
#include <optional>

//ROS2 Kinect2 Wrapper Node

class Kinect2Node : public rclcpp::Node
{
    private:

        //Tracking variables, used to determine if the device is connected and what streams are enabled
        bool deviceFound, enableRGB, enableDepth;

        //libfreenect2 variables
        libfreenect2::Freenect2Device *dev;
        libfreenect2::PacketPipeline *pipeline;
        std::unique_ptr<libfreenect2::Registration> registration;
        libfreenect2::Freenect2 freenect2;
        libfreenect2::FrameMap frames;
        std::string serial;

        //libfreenect2 variables that don't have default constructors
        std::optional<libfreenect2::Frame> undistorted, registered;
        std::optional<libfreenect2::SyncMultiFrameListener> listener;

        //Camera calibration parameters
        std::vector<double> cameraInfoDMatrix, cameraInfoKMatrix, cameraInfoPMatrix, cameraInfoRMatrix;
        std::string kinect2_link_frame_id;
        double imageWidth, imageHeight, imageDepth;

        //ROS Publishers and Timer
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ir_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_rect_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_rect_pub_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        //Creates Pipeline Based on Enabled Features
        void createPacketPipeline(std::string pipelineType);

    public:
        Kinect2Node();
        ~Kinect2Node();
        void timerCallback();
};

#endif