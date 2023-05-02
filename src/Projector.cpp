#include "Projector.h"

std::vector<MappedPoint> Projector::pc2img(const sensor_msgs::msg::PointCloud2 &pc) const{
    pcl::PointCloud<pcl::PointXYZ> pointCloud = ros2pcl(pc);
    pcl::PointCloud<pcl::PointXYZ> computeCloud;

    pcl::transformPointCloud(pointCloud, computeCloud, transformMatrix);

    std::vector<MappedPoint> out;
    cv::Point2i currentPoint;

    for(auto point: computeCloud){
        if(point.z < 0)
            continue;

        currentPoint = cameraModel.project3dToPixel({point.x, point.y, point.z});
        if(numInRange(currentPoint.x, 0, cameraData.width) && numInRange(currentPoint.y, 0, cameraData.height))
            out.emplace_back(currentPoint, point);
    }

    return out;
}

void Projector::getTransform(const sensor_msgs::msg::PointCloud2 &pc, const sensor_msgs::msg::Image &img)
{
    std::string camera_transform = img.header.frame_id;
    std::string lidar_transform = pc.header.frame_id;

    try{
        transform = buff->lookupTransform(camera_transform, lidar_transform, tf2::TimePointZero).transform;
    } catch(const tf2::TransformException & ex){
        RCLCPP_WARN(this->get_logger(),"Can't compute transform between %s and %s \n %s", camera_transform.c_str(), lidar_transform.c_str(), ex.what());
        transform = geometry_msgs::msg::Transform();
    }

    transformMatrix = tf2::transformToEigen(transform).matrix();
    inverseTransformMatrix = transformMatrix.inverse();
    
}

void Projector::setCameraInfo(const sensor_msgs::msg::CameraInfo &camInfo)
{
    cameraData = camInfo;
    cameraModel.fromCameraInfo(camInfo);
}

pcl::PointCloud<pcl::PointXYZ> Projector::ros2pcl(const sensor_msgs::msg::PointCloud2 &pc) const{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(pc, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> out_cloud;
    pcl::fromPCLPointCloud2(pcl_pc2,out_cloud);
    return out_cloud;
}

bool Projector::numInRange(int point, int lower, int upper) const{
    return (point <= upper && point >= lower);
}
