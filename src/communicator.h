#pragma once
#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <vector>
#include "ImageRegion.h"
#include "ImageSource.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <log4cxx/logger.h>
#include <opencv2/opencv.hpp>
#include "segmentation.h"
#include <tf/transform_listener.h>
#include <segmentation/imageRoiAction.h>
#include <actionlib/server/simple_action_server.h>

class Communicator {
public:
    Communicator(const Segmentation& seg);
    bool is_published(string req_topic);
    void depth_cb(const sensor_msgs::PointCloud2ConstPtr& cloud);
    void rgb_cb(const sensor_msgs::ImageConstPtr& rgb);
    void segment_cb(const segmentation::imageRoiGoalConstPtr& goal);
    void publishResults(ImageSource::Ptr& image, vector<ImageRegion::Ptr>& candidates, vector<Surface>& tables);
    segmentation::imageRoiResult publishRoi(vector<ImageRegion::Ptr>& candidates, ImageSource::Ptr& image);
    void publishClouds(ImageSource::Ptr& image);
    void publishSupportPlanes(vector<Surface>& tables);

protected:
   ros::NodeHandle node;
    actionlib::SimpleActionServer<segmentation::imageRoiAction> server;
private:
   ros::Subscriber segment_sub;
   ros::Subscriber depth_sub;
   ros::Subscriber rgb_sub;

   ros::Publisher image_pub;
   ros::Publisher cloud_pub;
   ros::Publisher bbox_pub;
   ros::Publisher plane_pub;


   tf::TransformListener tfListener;

   const string topic_d = "/xtion/depth/points";
   const string topic_rgb = "/xtion/rgb/image_raw";
   const string topic_pub_image = "/image";
   const string topic_pub_cloud = "/cloud";
   const string topic_pub_bbox = "/boundingBox";

   const uint TIMEOUT = 120;

   bool got_img;
   bool got_cloud;

   Segmentation seg;

   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
   cv::Mat image_;
   static const log4cxx::LoggerPtr logger;

};
#endif
