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
#include <object_tracking_msgs/Classify.h>
#include <object_tracking_msgs/RecognizeObjects.h>
#include <object_tracking_msgs/DetectPlanes.h>
#include <grasping_msgs/Object.h>

class Communicator {
public:
    Communicator(const Segmentation& seg);
    bool is_published(string req_topic);
    void depth_cb(const sensor_msgs::PointCloud2ConstPtr& cloud);
    void rgb_cb(const sensor_msgs::ImageConstPtr& rgb);
    bool recognize(object_tracking_msgs::RecognizeObjects::Request &req, object_tracking_msgs::RecognizeObjects::Response &res);
    vector<sensor_msgs::Image> getRoi(vector<ImageRegion::Ptr>& candidates, ImageSource::Ptr& image);
    bool get_segments(planning_scene_manager_msgs::Segmentation::Request &req, planning_scene_manager_msgs::Segmentation::Response &res);
    void clear_segments();
    bool detect_planes(object_tracking_msgs::DetectPlanes::Request &req, object_tracking_msgs::DetectPlanes::Response &res);

protected:
   ros::NodeHandle node;
private:
   ros::Subscriber segment_sub;
   ros::Subscriber depth_sub;
   ros::Subscriber rgb_sub;

    ros::Publisher object_pub;
    ros::Publisher segmented_cloud_pub;
    ros::Publisher table_cloud_pub;

   ros::ServiceClient classify_client;
   ros::ServiceServer segment_service;
   ros::ServiceServer recognize_service;
   ros::ServiceServer planes_service;
   ros::Publisher image_path_pub;


   tf::TransformListener tfListener;

   const string topic_d = "/xtion/depth/points";
   const string topic_rgb = "/xtion/rgb/image_raw";
   const string topic_pub_objects = "/classObjects";

   const uint TIMEOUT = 120;

   bool got_img;
   bool got_cloud;

    int num_objects;

   Segmentation seg;
   vector<grasping_msgs::Object> objects;
   vector<grasping_msgs::Object> support_planes;
   vector<string> config_names;

   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
   cv::Mat image_;
   static const log4cxx::LoggerPtr logger;

   void calcSupportPlanes(vector<Surface>& tables);

};
#endif
