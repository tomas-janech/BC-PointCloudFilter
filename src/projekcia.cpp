#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>

#include "Projector.h"

using PointType = pcl::PointXYZ;

class pc2image: public rclcpp::Node{
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr kalibracia;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr InputPC;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr FilteredPC;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr FilteredPCremoved;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr CameraImage;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ProjectedImage;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ProjectedOverlay;

    std::vector<int> segClasses;

    sensor_msgs::msg::Image lastRosImage;

    cv::Mat projected_points = cv::Mat::zeros(480,640,CV_8UC3);
    cv::Mat last_camera_image = cv::Mat::zeros(480,640,CV_8UC3);
    cv::Mat segImage = cv::Mat::ones(480,640,CV_8U);

    Projector projection;

    bool out_removed = false;

    void msgRecieved(const sensor_msgs::msg::CameraInfo &msg){
        projection.setCameraInfo(msg);
    }

    void ImgReceived(const sensor_msgs::msg::Image &msg){
        lastRosImage = msg;
        cv_bridge::CvImagePtr imgConv;
        try{
            imgConv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception &e){
            RCLCPP_ERROR(this->get_logger(),"Error loading image");
        }
        try
        {
            last_camera_image = imgConv->image;
        }
        catch(const cv::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),"CV error: %s", e.what());
        }
        
        
    }

    void projekcia(const sensor_msgs::msg::PointCloud2 &msg){

        projected_points = cv::Mat::zeros(projected_points.rows, projected_points.cols, CV_8UC3);

        projection.getTransform(msg, lastRosImage);
        auto points = projection.pc2img(msg);
        pcl::PointCloud<pcl::PointXYZ> outPc;
        pcl::PointCloud<pcl::PointXYZ> removedOutPc;

        for(auto point: points){
            cvDrawCircle(projected_points, point.mapped, 2, {0,0,255});

            //int pointKey = segImage.at<int>(point.mapped);

            /* if(std::find(segClasses.begin(),segClasses.end(),pointKey) != segClasses.end())
                outPc.emplace_back(point.original);
            else if(out_removed)
                removedOutPc.emplace_back(point.original); */

            outPc.emplace_back(point.original);

        }
        outPc.emplace_back(1,1,1);

        sensor_msgs::msg::PointCloud2 outRosPc;
        pcl2ros(outPc,outRosPc);
        outRosPc.header.frame_id = "orpheus_1_velodyne_1";
        FilteredPC->publish(outRosPc);

        sensor_msgs::msg::PointCloud2 outRemovedRosPc;
        pcl2ros(removedOutPc,outRemovedRosPc);
        FilteredPCremoved->publish(outRemovedRosPc);

        // TODO: publish camera with projected points 
        publishCVimage(projected_points, sensor_msgs::image_encodings::BGR8, ProjectedImage);
        publishCVimage(last_camera_image, sensor_msgs::image_encodings::BGR8, ProjectedOverlay);
    }

    void publishCVimage(cv::Mat &mat, std::string encoding, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher){

        cv_bridge::CvImage cv_img;

        try
        {
            cv_img.image = mat;
            cv_img.encoding = encoding;
        }
        catch(const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(),"Couldn't publish image: %s", e.what());
        }
        
        
        

        sensor_msgs::msg::Image::SharedPtr ImagePublish = cv_img.toImageMsg();

        publisher->publish(*ImagePublish);
    }

    void cvDrawCross(cv::Mat &image, const cv::Point &center, const int size, const cv::Scalar &color){
    cv::line(image,
            cv::Point(center.x - size, center.y - size),
            cv::Point(center.x + size, center.y + size),
            color);
    cv::line(image,
            cv::Point(center.x + size, center.y - size),
            cv::Point(center.x - size, center.y + size),
            color);
    return;
    }

    void cvDrawCircle(cv::Mat &image, const cv::Point &center, const int size, const cv::Scalar &color){
        cv::circle(image, center, size, color, -1);
    }

    void pcl2ros(const pcl::PointCloud<pcl::PointXYZ> &pc, sensor_msgs::msg::PointCloud2 &outMsg){
        pcl::PCLPointCloud2 PC_to_PCmsg;
        pcl::toPCLPointCloud2(pc, PC_to_PCmsg);
        pcl_conversions::fromPCL(PC_to_PCmsg, outMsg);
        return;
    };

public:
    pc2image():Node("projekcia_obrazu"){
        this->declare_parameter<std::string>("point_cloud_topic","null");
        this->declare_parameter<std::string>("camera_topic","null");
        this->declare_parameter<std::string>("segmentation_topic","null");
        this->declare_parameter<std::string>("output_cloud_name","null");  
        this->declare_parameter<std::vector<std::string>>("segmentation_classes",{"null"});
        this->declare_parameter<std::vector<int>>("segmentation_codes",{-1});
        this->declare_parameter<std::vector<std::string>>("remove_classes",{"null"});
        this->declare_parameter<bool>("output_removed",false);
        this->declare_parameter<std::string>("output_removed_cloud_name","null");

        segClasses = {0,1,2,3,4,5,6,7,13,14,15,16,17,18,19};
        out_removed = true;

        kalibracia = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/orpheus_1/sensors/realsense_2/camera_info",
            10,
            std::bind(&pc2image::msgRecieved, this, std::placeholders::_1)
        );
        InputPC = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/orpheus_1/sensors/velodyne_1/velodyne_points",
            10,
            std::bind(&pc2image::projekcia, this, std::placeholders::_1)
        );
        CameraImage = this->create_subscription<sensor_msgs::msg::Image>(
            "/orpheus_1/sensors/realsense_2/image_raw",
            10,
            std::bind(&pc2image::ImgReceived, this, std::placeholders::_1)
        );
        ProjectedImage = this->create_publisher<sensor_msgs::msg::Image>("/projected_image", 10);
        ProjectedOverlay = this->create_publisher<sensor_msgs::msg::Image>("/projected_overlay", 10);
        FilteredPC = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filteredCloud", 10);
        FilteredPCremoved = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filteredCloudRemoved", 10);
    };
};


int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pc2image>());
    rclcpp::shutdown();
    return 0;
}