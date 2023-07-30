#include "kinect2_driver.hpp"

Kinect2Node::Kinect2Node() : Node("kinect2_node"), listener(defaultSyncType)
{
    this->deviceFound = false;
    this->pipeline = 0;

    std::string pipelineType = this->declare_parameter<std::string>("pipeline_type", "cpu");

    try
    {
        this->createPacketPipeline(pipelineType);
    }
    catch(const std::invalid_argument& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid pipeline type: %s", e.what());
        throw e;
    }

    if(this->freenect2.enumerateDevices() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "No Kinect devices found!");
        throw std::runtime_error("No Kinect devices found!"); 
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Found %d Kinect devices", this->freenect2.enumerateDevices());
    }

    this->serial = this->freenect2.getDefaultDeviceSerialNumber();
    
    if(this->pipeline)
    {
        this->dev = this->freenect2.openDevice(this->serial, this->pipeline);
    }
    else
    {
        this->dev = this->freenect2.openDevice(this->serial);
    }

    if(this->dev == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open Kinect device!");
        throw std::runtime_error("Failed to open Kinect device!");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Opened Kinect device");
    }

    RCLCPP_INFO(this->get_logger(), "Starting Kinect device");

    this->enableRGB = this->declare_parameter<bool>("enable_rgb", true);
    this->enableDepth = this->declare_parameter<bool>("enable_depth", true);

    int types = 0;
    if(enableRGB)
        types |= libfreenect2::Frame::Color;
    if(enableDepth)
        types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    if(types == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "No data types enabled!");
        throw std::invalid_argument("No data types enabled!");
    }

    this->listener.emplace(types);

    this->dev->setColorFrameListener(&(*this->listener));
    this->dev->setIrAndDepthFrameListener(&(*this->listener));
    
    if(!this->dev->start())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to start Kinect device!");
        throw std::runtime_error("Failed to start Kinect device!");
    }

    RCLCPP_INFO(this->get_logger(), "Started Kinect device");

    this->registration = std::make_unique<libfreenect2::Registration>(this->dev->getIrCameraParams(), this->dev->getColorCameraParams());

    // Initialize publishers
    this->rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/kinect2/rgb/raw", 10);
    this->depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/kinect2/depth/raw", 10);
    this->ir_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/kinect2/ir/raw", 10);
    this->depth_rect_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/kinect2/rgbd/depth_rect", 10);
    this->rgb_rect_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/kinect2/rgbd/rgb_rect", 10);
    this->camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/kinect2/rgbd/camera_info", 10);

    // Initialize timer
    double updateRate = this->declare_parameter<double>("update_rate", 30.0);
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / updateRate)),
                                     std::bind(&Kinect2Node::timerCallback, this));

    this->cameraInfoDMatrix = this->declare_parameter<std::vector<double>>("camera_info_d", {0.012947732047700113, -0.016278227805096242, 0.0020719045565612245, -0.0012254560479249, 0.0});
    this->cameraInfoKMatrix = this->declare_parameter<std::vector<double>>("camera_info_k", {362.02061428537024, 0.0, 210.76517637501865, 0.0, 405.17059240104345, 211.97321671296146, 0.0, 0.0, 1.0});
    this->cameraInfoRMatrix = this->declare_parameter<std::vector<double>>("camera_info_r", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0 ,0.0, 1.0});
    this->cameraInfoPMatrix = this->declare_parameter<std::vector<double>>("camera_info_p", {361.9730224609375, 0.0, 209.32856710752822, 0.0, 0.0, 405.724365234375, 212.33942579256836, 0.0, 0.0, 0.0, 1.0, 0.0});
    this->kinect2_link_frame_id = this->declare_parameter<std::string>("kinect2_link_frame_id", "kinect2_link");

    this->imageWidth = this->declare_parameter<double>("image_width", 512.0);
    this->imageHeight = this->declare_parameter<double>("image_height", 424.0);
    this->imageDepth = this->declare_parameter<double>("image_depth", 4.0);

    this->undistorted.emplace(this->imageWidth, this->imageHeight, this->imageDepth);
    this->registered.emplace(this->imageWidth, this->imageHeight, this->imageDepth);

    this->deviceFound = true;

    RCLCPP_INFO(this->get_logger(), "Kinect2Node initialized");
}

Kinect2Node::~Kinect2Node()
{
    RCLCPP_INFO(this->get_logger(), "Stopping Kinect device");

    //Close device if it was initialized
    if(this->deviceFound)
    {
        (*this->listener).release(this->frames);
        this->dev->stop();
        this->dev->close();
    }

    RCLCPP_INFO(this->get_logger(), "Stopped Kinect device");
}

void Kinect2Node::timerCallback()
{
    if(!(*this->listener).waitForNewFrame(this->frames, 10*1000))
    {
        RCLCPP_ERROR(this->get_logger(), "Timed out waiting for Kinect data!");
        return;
    }

    libfreenect2::Frame *rgb, *ir, *depth;
    if(this->enableRGB)
    {
        rgb = frames[libfreenect2::Frame::Color];
    }
    if(this->enableDepth)
    {
        ir = frames[libfreenect2::Frame::Ir];
        depth = frames[libfreenect2::Frame::Depth];
    }

    if(this->enableRGB && this->enableDepth)
    {
        this->registration->apply(rgb, depth, &(*this->undistorted), &(*this->registered));
    }

    if(this->enableRGB)
    {
        //Publish RGB image
        auto rgb_msg = std::make_unique<sensor_msgs::msg::Image>();
        rgb_msg->header.frame_id = this->kinect2_link_frame_id;
        rgb_msg->header.stamp = this->now();
        rgb_msg->height = rgb->height;
        rgb_msg->width = rgb->width;
        rgb_msg->step = rgb->bytes_per_pixel * rgb->width;
        rgb_msg->encoding = sensor_msgs::image_encodings::BGRA8;
        rgb_msg->data.resize(rgb_msg->step * rgb_msg->height);
        memcpy(&rgb_msg->data[0], rgb->data, rgb_msg->step * rgb_msg->height);

        this->rgb_pub_->publish(std::move(rgb_msg));
    }

    if(this->enableDepth)
    {
        //Publish IR image
        auto ir_msg = std::make_unique<sensor_msgs::msg::Image>();
        ir_msg->header.frame_id = this->kinect2_link_frame_id;
        ir_msg->header.stamp = this->now();
        ir_msg->height = ir->height;
        ir_msg->width = ir->width;
        ir_msg->step = ir->bytes_per_pixel * ir->width;
        ir_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        ir_msg->data.resize(ir_msg->step * ir_msg->height);
        memcpy(&ir_msg->data[0], ir->data, ir_msg->step * ir_msg->height);

        this->ir_pub_->publish(std::move(ir_msg));

        //Publish depth image

        auto depth_msg = std::make_unique<sensor_msgs::msg::Image>();
        depth_msg->header.frame_id = this->kinect2_link_frame_id;
        depth_msg->header.stamp = this->now();
        depth_msg->height = depth->height;
        depth_msg->width = depth->width;
        depth_msg->step = depth->bytes_per_pixel * depth->width;
        depth_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        depth_msg->data.resize(depth_msg->step * depth_msg->height);
        memcpy(&depth_msg->data[0], depth->data, depth_msg->step * depth_msg->height);

        this->depth_pub_->publish(std::move(depth_msg));
    }

    if(this->enableRGB && this->enableDepth)
    {
        //Publish Undistorted Depth Image
        auto depth_rect_msg = std::make_unique<sensor_msgs::msg::Image>();
        depth_rect_msg->header.frame_id = this->kinect2_link_frame_id;
        depth_rect_msg->header.stamp = this->now();
        depth_rect_msg->height = (this->undistorted)->height;
        depth_rect_msg->width = (this->undistorted)->width;
        depth_rect_msg->step = (this->undistorted)->bytes_per_pixel * (this->undistorted)->width;
        depth_rect_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        depth_rect_msg->data.resize(depth_rect_msg->step * depth_rect_msg->height);
        memcpy(&depth_rect_msg->data[0], (this->undistorted)->data, depth_rect_msg->step * depth_rect_msg->height);

        this->depth_rect_pub_->publish(std::move(depth_rect_msg));

        //Publish Undistorted RGB Image

        auto rgb_rect_msg = std::make_unique<sensor_msgs::msg::Image>();
        rgb_rect_msg->header.frame_id = this->kinect2_link_frame_id;
        rgb_rect_msg->header.stamp = this->now();
        rgb_rect_msg->height = (this->registered)->height;
        rgb_rect_msg->width = (this->registered)->width;
        rgb_rect_msg->step = (this->registered)->bytes_per_pixel * (this->registered)->width;
        rgb_rect_msg->encoding = sensor_msgs::image_encodings::BGRA8;
        rgb_rect_msg->data.resize(rgb_rect_msg->step * rgb_rect_msg->height);
        memcpy(&rgb_rect_msg->data[0], (this->registered)->data, rgb_rect_msg->step * rgb_rect_msg->height);

        this->rgb_rect_pub_->publish(std::move(rgb_rect_msg));
    }

    //Publish camera info
    auto camera_info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    camera_info_msg->header.frame_id = this->kinect2_link_frame_id;
    camera_info_msg->header.stamp = this->now();
    camera_info_msg->width = depth->width;
    camera_info_msg->height = depth->height;
    camera_info_msg->distortion_model = "plumb_bob";
    camera_info_msg->d.resize(5);
    std::copy(this->cameraInfoDMatrix.begin(), this->cameraInfoDMatrix.end(), camera_info_msg->d.begin());
    std::copy(this->cameraInfoKMatrix.begin(), this->cameraInfoKMatrix.end(), camera_info_msg->k.begin());
    std::copy(this->cameraInfoRMatrix.begin(), this->cameraInfoRMatrix.end(), camera_info_msg->r.begin());
    std::copy(this->cameraInfoPMatrix.begin(), this->cameraInfoPMatrix.end(), camera_info_msg->p.begin());

    this->camera_info_pub_->publish(std::move(camera_info_msg));

    (*this->listener).release(this->frames);
}

void Kinect2Node::createPacketPipeline(std::string pipelineType)
{
    if(pipelineType == "cpu")
    {
        RCLCPP_INFO(this->get_logger(), "Using CPU pipeline");
        if(!this->pipeline)
            this->pipeline = new libfreenect2::CpuPacketPipeline();
    }
    else if(pipelineType == "gl")
    {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        if(!this->pipeline)
            this->pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
        RCLCPP_ERROR(this->get_logger(), "OpenGL pipeline is not supported!");
        throw std::invalid_argument("OpenGL pipeline is not supported!");
#endif
    }
    else if(pipelineType == "cl")
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        if(!this->pipeline)
            this->pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
        RCLCPP_ERROR(this->get_logger(), "OpenCL pipeline is not supported!");
        throw std::invalid_argument("OpenCL pipeline is not supported!");
#endif
    }
    else if(pipelineType == "clkde")
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        if(!this->pipeline)
            this->pipeline = new libfreenect2::OpenCLKdePacketPipeline();
#else
        RCLCPP_ERROR(this->get_logger(), "OpenCL Kde pipeline is not supported!");
        throw std::invalid_argument("OpenCL Kde pipeline is not supported!");
#endif
    }
    else if(pipelineType == "cuda")
    {
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
        if(!this->pipeline)
            this->pipeline = new libfreenect2::CudaPacketPipeline();
#else
        RCLCPP_ERROR(this->get_logger(), "CUDA pipeline is not supported!");
        throw std::invalid_argument("CUDA pipeline is not supported!");
#endif
    }
    else if(pipelineType == "cudakde")
    {
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
        if(!this->pipeline)
            this->pipeline = new libfreenect2::CudaKdePacketPipeline();
#else
        RCLCPP_ERROR(this->get_logger(), "CUDA Kde pipeline is not supported!");
        throw std::invalid_argument("CUDA Kde pipeline is not supported!");
#endif
    }

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Kinect2Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}