#pragma once

// ROS C++ package
#include <rclcpp/rclcpp.hpp>

// Sensor msgs
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

// Geometry messages
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>
#include <tf2_eigen/tf2_eigen.hpp>

// Image geomerty
#include <image_geometry/pinhole_camera_model.h>

// PCL packages
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/common/transforms.h>

// C++ std
#include <vector>
#include <string>
#include <initializer_list>

struct MappedPoint{
    cv::Point2i mapped;
    pcl::PointXYZ original;

    MappedPoint(cv::Point2i img, pcl::PointXYZ spc):mapped(img), original(spc){};
};

class Projector: public rclcpp::Node{

    // Camera related
    sensor_msgs::msg::CameraInfo cameraData;
    image_geometry::PinholeCameraModel cameraModel;

    // Camera->Lidar geometry
    geometry_msgs::msg::Transform transform;
    std::unique_ptr<tf2_ros::Buffer> buff;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    void filterPc(const pcl::PointCloud<pcl::PointXYZ> &inPC, pcl::PointCloud<pcl::PointXYZ> &outPC) const;

    /**
     * \brief Converts ros PC2 to PCL PC
     * \author Janech Tomas
     * \param[in] pc ros pointcloud2
    */
    pcl::PointCloud<pcl::PointXYZ> ros2pcl(const sensor_msgs::msg::PointCloud2 &pc) const;

    /**
     * \brief Helper, checks wheter int is in range
     * \author Janeh Tomas
     * \param[in] point checked point
     * \param[in] lower lower interval limit
     * \param[in] upper upper interval limit
     * \returns true, if point is in interval
     * \returns false, otherwise
    */
    bool numInRange(int point, int lower, int upper) const;

public:

    Eigen::Matrix4d transformMatrix;
    Eigen::Matrix4d inverseTransformMatrix;

    Projector():Node("projektor"), cameraModel(), transformMatrix() {
        buff = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buff);
    };

    /**
     * \brief Projects pointcloud to image
     * \details Projects PC to image specified by CameraInfo message. Uses camera->lidar transform
     * \author Janech Tomas
     * \param[in] pc PointCloud to be transorfmed
     * \returns vector of points that lay inside of camera sensor
    */
    std::vector<MappedPoint> pc2img(const sensor_msgs::msg::PointCloud2 &pc) const;

    /**
     * \brief Calculates lidar->camera transform
     * \author Janech Tomas
     * \param[in] pc PointCloud message, used for tf
     * \param[in] img Image message, used for tf 
    */
    void getTransform(const sensor_msgs::msg::PointCloud2 &pc, const sensor_msgs::msg::Image &img);

    /**
     * \brief Set camera parameters
     * \details Sets camera paramteres from camerainfo message. Camera needs to be calibrated.
     * \author Janech Tomas
     * \param[in] camInfo Camera calibration data  
    */
    void setCameraInfo(const sensor_msgs::msg::CameraInfo &camInfo);
};

