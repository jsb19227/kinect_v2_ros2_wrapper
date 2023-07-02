#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string.h>

unsigned int syncType = libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;

class KinectNode : public rclcpp::Node
{
public:
  KinectNode() : Node("kinect_node")
  {

    pipeline = std::unique_ptr<libfreenect2::PacketPipeline>(new libfreenect2::OpenGLPacketPipeline());

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

    dev = std::unique_ptr<libfreenect2::Freenect2Device>(freenect2.openDevice(serial, pipeline.get()));

    if(dev == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open Kinect device!");
      return;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Opened Kinect device");
    }

    // unsigned int syncType = libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    // listener = libfreenect2::SyncMultiFrameListener(syncType);

    RCLCPP_INFO(this->get_logger(), "Starting Kinect device");

    dev.get()->setColorFrameListener(&listener);
    dev.get()->setIrAndDepthFrameListener(&listener);

    dev.get()->start();

    RCLCPP_INFO(this->get_logger(), "Started Kinect device");

    registration = std::unique_ptr<libfreenect2::Registration>(new libfreenect2::Registration(dev.get()->getIrCameraParams(), dev.get()->getColorCameraParams()));
    // undistorted = libfreenect2::Frame(512, 424, 4);
    // registered = libfreenect2::Frame(512, 424, 4);


    // Initialize publishers
    rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("rgb", 10);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth", 10);
    ir_pub_ = this->create_publisher<sensor_msgs::msg::Image>("ir", 10);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

    // Set up publishing rate
    double rate_hz = 30.0;
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz)),
                                     std::bind(&KinectNode::publishData, this));
  }

  ~KinectNode()
  {
    RCLCPP_INFO(this->get_logger(), "Stopping Kinect device");
    dev->stop();
  }

private:
  void publishData()
  {
    RCLCPP_INFO(this->get_logger(), "Publishing data");
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

    auto camera_info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    camera_info_msg->header.stamp = this->now();
    camera_info_msg->header.frame_id = "kinect_link";
    camera_info_msg->width = rgb->width;
    camera_info_msg->height = rgb->height;
    camera_info_msg->distortion_model = "plumb_bob";
    camera_info_msg->d.resize(5);
    camera_info_msg->d = {0.0663631194076439, -0.050010995621799885, -0.009347016576573004, -0.006714620351360462, 0.0};
    camera_info_msg->k = {1084.9090564027433, 0.0, 938.0495572093841, 0.0, 1091.6943330193576, 512.5782019120095, 0.0, 0.0, 1.0};
    camera_info_msg->r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    camera_info_msg->p = {1111.7230224609375, 0.0, 918.7707584382588, 0.0, 0.0, 1124.953125, 499.42883756104857, 0.0, 0.0, 0.0, 1.0, 0.0};

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


    auto pointcloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    auto rgb_rect_msg = std::make_unique<sensor_msgs::msg::Image>();
    rgb_rect_msg->header.stamp = this->now();
    rgb_rect_msg->width = depth->width;
    rgb_rect_msg->height = depth->height;
    rgb_rect_msg->step = 3 * depth->width;
    rgb_rect_msg->encoding = sensor_msgs::image_encodings::BGR8;
    rgb_rect_msg->data.resize(3 * depth->width * depth->height);


    for(unsigned long i = 0; i < depth->width; i++)
    {
        for(unsigned long  j = 0; j < depth->height; j++)
        {
            RCLCPP_INFO(this->get_logger(), "i: %d, j: %d", i, j);
            float x, y, z, rgb;
            registration->getPointXYZRGB(&undistorted, &registered, j, i, x, y, z, rgb);
            const uint8_t* pixel = reinterpret_cast<uint8_t*>(&rgb);
            uint8_t b = pixel[0];
            uint8_t g = pixel[1];
            uint8_t r = pixel[2];

            pcl::PointXYZRGB point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.r = r;
            point.g = g;
            point.b = b;

            rgb_rect_msg->data[j * depth->height + i] = pixel[0];

            pcl_cloud->points.push_back(point);
        }
    }

    pcl::toROSMsg(*pcl_cloud, *pointcloud_msg);

    pointcloud_msg->header.frame_id = "kinect_link";
    pointcloud_msg->header.stamp = this->now();
    pointcloud_msg->is_dense = false;
    pointcloud_msg->is_bigendian = false;


    rgb_pub_->publish(std::move(image_msg));
    depth_pub_->publish(std::move(depth_msg));
    rgb_rect_pub_->publish(std::move(rgb_rect_msg));
    camera_info_pub_->publish(std::move(camera_info_msg));
    pointcloud_pub_->publish(std::move(pointcloud_msg));
    ir_pub_->publish(std::move(ir_msg));

    listener.release(frames);

  }

    std::unique_ptr<libfreenect2::Freenect2Device> dev;
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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinectNode>());
  rclcpp::shutdown();
  return 0;
}
