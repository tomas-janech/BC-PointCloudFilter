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

class pc2image: public rclcpp::Node{
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr calibTopic;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr InputPC;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr FilteredPC;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr FilteredPCremoved;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr SegImageRos;

    std::vector<int> segClasses;
    cv::Mat segImage = cv::Mat::zeros(480,640,CV_8U);

    Projector projection;

    void CameraInfoReceive(const sensor_msgs::msg::CameraInfo &msg){
        projection.setCameraInfo(msg);
    }

    void SegImgReceive(const sensor_msgs::msg::Image &msg){
        segImage = cv::Mat::zeros(480,640,CV_8U);
        cv_bridge::CvImagePtr imgConv;
        try{
            imgConv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch(cv_bridge::Exception &e){
            RCLCPP_ERROR(this->get_logger(),"Error loading image");
        }
        try
        {
            segImage = imgConv->image;
        }
        catch(const cv::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),"CV error: %s", e.what());
        }
        
        
    }

    void filter(const sensor_msgs::msg::PointCloud2 &msg){

        pcl::PointCloud<pcl::PointXYZ> outPc;
        pcl::PointCloud<pcl::PointXYZ> removedOutPc;

        // Project PointCloud onto Image
        projection.getTransform(msg.header.frame_id);
        auto points = projection.pc2img(msg);
        
        // Filter projected points
        for(auto point: points){
            int pointKey = segImage.at<uint8_t>(point.mapped);

            if(std::find(segClasses.begin(),segClasses.end(),pointKey) != segClasses.end())
                outPc.emplace_back(point.original);
            else if(this->get_parameter("output_removed").get_value<bool>())
                removedOutPc.emplace_back(point.original);
        }

        // Output filtered PointClouds

        pcl::transformPointCloud(outPc,outPc,projection.inverseTransformMatrix);  

        sensor_msgs::msg::PointCloud2 outRosPc;
        pcl2ros(outPc,outRosPc);
        outRosPc.header.frame_id = msg.header.frame_id;
        FilteredPC->publish(outRosPc);

        pcl::transformPointCloud(removedOutPc,removedOutPc,projection.inverseTransformMatrix);

        sensor_msgs::msg::PointCloud2 outRemovedRosPc;
        pcl2ros(removedOutPc,outRemovedRosPc);
        outRemovedRosPc.header.frame_id = msg.header.frame_id;
        FilteredPCremoved->publish(outRemovedRosPc);
    }

    void pcl2ros(const pcl::PointCloud<pcl::PointXYZ> &pc, sensor_msgs::msg::PointCloud2 &outMsg){
        pcl::PCLPointCloud2 PC_to_PCmsg;
        pcl::toPCLPointCloud2(pc, PC_to_PCmsg);
        pcl_conversions::fromPCL(PC_to_PCmsg, outMsg);
        return;
    };

public:
    pc2image():Node("PC_filter"){

        // Parameters
        this->declare_parameter<std::string>("point_cloud_topic","null");
        this->declare_parameter<std::string>("camera_topic","null");
        this->declare_parameter<std::string>("segmentation_topic","null");
        this->declare_parameter<std::string>("output_cloud_name","null");  
        this->declare_parameter<std::vector<std::string>>("segmentation_classes",{"null"});
        this->declare_parameter<std::vector<int>>("segmentation_codes",{-1});
        this->declare_parameter<std::vector<std::string>>("remove_classes",{"null"});
        this->declare_parameter<bool>("output_removed",false);
        this->declare_parameter<std::string>("output_removed_cloud_name","null");


        // Filter classes decalration
        auto segmentation_classes_codes = this->get_parameter("segmentation_codes").get_value<std::vector<int>>();
        auto segmentation_classes = this->get_parameter("segmentation_classes").get_value<std::vector<std::string>>();
        auto segmentation_classes_remove = this->get_parameter("remove_classes").get_value<std::vector<std::string>>();
        std::multimap<std::string, int> classes;

        if(segmentation_classes.size() != segmentation_classes_codes.size())
            RCLCPP_WARN(this->get_logger(), "Ignoring last %ld segmentation classes!", segmentation_classes.size() - segmentation_classes_codes.size());

        for (size_t i = 0; i < segmentation_classes_codes.size() && i < segmentation_classes.size(); i++)
            classes.insert({segmentation_classes[i],segmentation_classes_codes[i]});

        for (size_t i = 0; i < segmentation_classes_remove.size(); i++)
            classes.erase(segmentation_classes_remove[i]);
        
        for(const auto &[key, val]: classes)
            segClasses.emplace_back(val);


        // Subscribers
        calibTopic = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            this->get_parameter("camera_topic").get_value<std::string>() + "/camera_info",
            10,
            std::bind(&pc2image::CameraInfoReceive, this, std::placeholders::_1)
        );
        InputPC = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("point_cloud_topic").get_value<std::string>(),
            10,
            std::bind(&pc2image::filter, this, std::placeholders::_1)
        );
        SegImageRos = this->create_subscription<sensor_msgs::msg::Image>(
            this->get_parameter("segmentation_topic").get_value<std::string>(),
            10,
            std::bind(&pc2image::SegImgReceive, this, std::placeholders::_1));


        // Publishers
        FilteredPC = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->get_parameter("output_cloud_name").get_value<std::string>(), 10);
        FilteredPCremoved = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->get_parameter("output_removed_cloud_name").get_value<std::string>(), 10);
    };
};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pc2image>());
    rclcpp::shutdown();
    return 0;
}