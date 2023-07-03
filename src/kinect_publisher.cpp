#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <string.h>
#include <opencv2/opencv.hpp>

#include <iostream>

unsigned int syncType = libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;

class KinectNode : public rclcpp::Node
{
public:
  KinectNode() : Node("kinect_node")
  {
    pipeline = std::make_unique<libfreenect2::OpenGLPacketPipeline>();

    if(freenect2.enumerateDevices() == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "No Kinect devices found!");
      return;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Found %d Kinect devices", freenect2.enumerateDevices());
    }

    serial = freenect2.getDefaultDeviceSerialNumber();
    dev = freenect2.openDevice(serial);

    if(dev == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open Kinect device!");
      return;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Opened Kinect device");
    }

    RCLCPP_INFO(this->get_logger(), "Starting Kinect device");

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->start();

    RCLCPP_INFO(this->get_logger(), "Started Kinect device");

    registration = std::make_unique<libfreenect2::Registration>(dev->getIrCameraParams(), dev->getColorCameraParams());

    // Initialize publishers
    rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/kinect/rgb_raw", 10);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/kinect/depth_raw", 10);
    ir_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/kinect/ir_raw", 10);
    rgb_rect_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/kinect/rgb_depth", 10);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/kinect/depth_camera_info", 10);

    // Set up publishing rate
    double rate_hz = 30.0;
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz)),
                                     std::bind(&KinectNode::publishData, this));
  }

  ~KinectNode()
  {
    RCLCPP_INFO(this->get_logger(), "Stopping Kinect device");
    listener.release(frames);
    
    dev->stop();
    dev->close();

    RCLCPP_INFO(this->get_logger(), "Stopped Kinect device");
  }

private:
  void publishData()
  {
    if (!listener.waitForNewFrame(frames, 10*1000)) // 10 sconds
    {
      std::cout << "timeout!" << std::endl;
      return;
    }

    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];

    registration->apply(rgb, depth, &undistorted, &registered);

    auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
    image_msg->header.stamp = this->now();
    image_msg->width = rgb->width;
    image_msg->height = rgb->height;
    image_msg->step = rgb->bytes_per_pixel * rgb->width;
    image_msg->encoding = sensor_msgs::image_encodings::BGRA8;
    image_msg->data.resize(rgb->width * rgb->height * rgb->bytes_per_pixel);
    memcpy(image_msg->data.data(), rgb->data, rgb->width * rgb->height * rgb->bytes_per_pixel);

    cv::Mat original_image(rgb->height, rgb->width, CV_8UC4, rgb->data);
    cv::Mat resized_image;
    cv::Rect roi(350, 0, 1450, 1080);
    cv::Mat cropped_image = original_image(roi);
    cv::resize(cropped_image, resized_image, cv::Size(512, 424));

    auto rgb_rect_msg = std::make_unique<sensor_msgs::msg::Image>();
    rgb_rect_msg->header.stamp = this->now();
    rgb_rect_msg->header.frame_id = "kinect_link";
    rgb_rect_msg->width = depth->width;
    rgb_rect_msg->height = depth->height;
    rgb_rect_msg->step = 4 * depth->width;
    rgb_rect_msg->encoding = sensor_msgs::image_encodings::BGRA8;
    rgb_rect_msg->data.resize(depth->width * depth->height * 4); 
    memcpy(rgb_rect_msg->data.data(), resized_image.data, depth->width * depth->height * 4);

    auto camera_info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    camera_info_msg->header.stamp = this->now();
    camera_info_msg->header.frame_id = "kinect_link";
    camera_info_msg->width = depth->width;
    camera_info_msg->height = depth->height;
    camera_info_msg->distortion_model = "plumb_bob";
    camera_info_msg->d.resize(5);
    camera_info_msg->d = {0.012947732047700113, -0.016278227805096242, 0.0020719045565612245, -0.0012254560479249, 0.0};
    camera_info_msg->k = {362.02061428537024, 0.0, 210.76517637501865, 0.0, 405.17059240104345, 211.97321671296146, 0.0, 0.0, 1.0};
    camera_info_msg->r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    camera_info_msg->p = {361.9730224609375, 0.0, 209.32856710752822, 0.0, 0.0, 405.724365234375, 212.33942579256836, 0.0, 0.0, 0.0, 1.0, 0.0};

    auto depth_msg = std::make_unique<sensor_msgs::msg::Image>();
    depth_msg->header.stamp = this->now();
    depth_msg->width = depth->width;
    depth_msg->height = depth->height;
    depth_msg->step = depth->bytes_per_pixel * depth->width;
    depth_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    depth_msg->data.resize(depth->width * depth->height * depth->bytes_per_pixel);
    memcpy(depth_msg->data.data(), depth->data, depth->width * depth->height * depth->bytes_per_pixel);

    auto ir_msg = std::make_unique<sensor_msgs::msg::Image>();
    ir_msg->header.stamp = this->now();
    ir_msg->width = ir->width;
    ir_msg->height = ir->height;
    ir_msg->step = ir->bytes_per_pixel * ir->width;
    ir_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    ir_msg->data.resize(ir->width * ir->height * ir->bytes_per_pixel);
    memcpy(ir_msg->data.data(), ir->data, ir->width * ir->height * ir->bytes_per_pixel);


    rgb_pub_->publish(std::move(image_msg));
    depth_pub_->publish(std::move(depth_msg));
    rgb_rect_pub_->publish(std::move(rgb_rect_msg));
    camera_info_pub_->publish(std::move(camera_info_msg));
    ir_pub_->publish(std::move(ir_msg));

    listener.release(frames);

  }

    libfreenect2::Freenect2Device *dev;
    std::unique_ptr<libfreenect2::PacketPipeline> pipeline;
    libfreenect2::Freenect2 freenect2;
    std::string serial;

    libfreenect2::SyncMultiFrameListener listener = libfreenect2::SyncMultiFrameListener(syncType);

    libfreenect2::FrameMap frames;
    std::unique_ptr<libfreenect2::Registration> registration;
    libfreenect2::Frame undistorted = libfreenect2::Frame(512, 424, 4), registered = libfreenect2::Frame(512, 424, 4);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ir_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_rect_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KinectNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
