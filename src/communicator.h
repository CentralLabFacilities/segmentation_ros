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
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/simple_action_server.h>
#include <planning_scene_manager_msgs/Segmentation.h>
#include <planning_scene_manager_msgs/FittingConfig.h>
#include <object_recognition_msgs/segment.h>
#include <grasping_msgs/Object.h>

class Communicator {
public:
    Communicator(const Segmentation& seg);
    bool is_published(string req_topic);
    void depth_cb(const sensor_msgs::PointCloud2ConstPtr& cloud);
    void rgb_cb(const sensor_msgs::ImageConstPtr& rgb);
    bool segment_cb(object_recognition_msgs::segment::Request &req, object_recognition_msgs::segment::Response &res);
    void publishResults(ImageSource::Ptr& image, vector<ImageRegion::Ptr>& candidates, vector<Surface>& tables);
    vector<sensor_msgs::Image> publishRoi(vector<ImageRegion::Ptr>& candidates, ImageSource::Ptr& image);
    void publishClouds(ImageSource::Ptr& image);
    bool getSegments(planning_scene_manager_msgs::Segmentation::Request &req, planning_scene_manager_msgs::Segmentation::Response &res);
    //void publishSupportPlanes(vector<Surface>& tables);

protected:
   ros::NodeHandle node;
   //actionlib::SimpleActionServer<segmentation::imageRoiAction> server;
private:
   ros::Subscriber segment_sub;
   ros::Subscriber depth_sub;
   ros::Subscriber rgb_sub;

   ros::Publisher object_pub;

   ros::ServiceClient class_client;
   ros::ServiceServer segment_service;
   ros::ServiceServer planning_service;


   tf::TransformListener tfListener;

   const string topic_d = "/xtion/depth/points";
   const string topic_rgb = "/xtion/rgb/image_raw";
   const string topic_pub_objects = "/classObjects";

   const uint TIMEOUT = 120;

   bool got_img;
   bool got_cloud;

   Segmentation seg;
   vector<grasping_msgs::Object> objects;
   vector<grasping_msgs::Object> support_planes;
   planning_scene_manager_msgs::FittingConfig config;
   vector<planning_scene_manager_msgs::FittingConfig> configs;

   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
   cv::Mat image_;
   static const log4cxx::LoggerPtr logger;

};
#endif
