#include <stdio.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include "Surface.h"
#include "segmentation.h"
#include "communicator.h"
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/consoleappender.h>
#include <log4cxx/patternlayout.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace log4cxx;

const LoggerPtr Communicator::logger = Logger::getLogger("segmentation.communicator");

Communicator::Communicator(const Segmentation& segmentation): seg(segmentation), server(node, "segment", boost::bind(&Communicator::segment_cb, this, _1), false) {

    server.start();
    segment_sub = node.subscribe("segment", 10, &Communicator::segment_cb, this);
    LOG4CXX_DEBUG(logger, "subscribed to Topic segment\n");

    image_pub = node.advertise<cv_bridge::CvImage>(topic_pub_image, 10);
    cloud_pub = node.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(topic_pub_cloud, 10);
    bbox_pub = node.advertise<sensor_msgs::RegionOfInterest>(topic_pub_bbox, 10);
    LOG4CXX_DEBUG(logger, "created ROS topics " << topic_pub_image << ", " << topic_pub_cloud << ", " << topic_pub_bbox << "\n");
}

bool Communicator::is_published(string req_topic) {
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        if(info.name == req_topic)
            return true;
    }
    return false;
}

void Communicator::depth_cb(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    LOG4CXX_DEBUG(logger, "got cloud information.\n");
    cloud_ = PointCloud<PointXYZRGBA>::Ptr(new PointCloud<PointXYZRGBA>());
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_);

    depth_sub.shutdown();

    got_cloud = true;
}

void Communicator::rgb_cb(const sensor_msgs::ImageConstPtr& rgb) {
    LOG4CXX_DEBUG(logger, "got rgb image information.\n");
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::cvtColor(cv_ptr->image, image_, CV_BGR2RGB);
    rgb_sub.shutdown();

    got_img = true;
}

void Communicator::segment_cb(const segmentation::imageRoiGoalConstPtr& goal)
//void Communicator::segment_cb(const std_msgs::String::ConstPtr &msg)
{
    LOG4CXX_DEBUG(logger, "got segmentation request.\n");
    bool d = is_published(topic_d);
    bool rgb = is_published(topic_rgb);
    if (!d || !rgb) {
        LOG4CXX_WARN(logger, "neither " << topic_d << ", nor " << topic_rgb << " are available!\n");
    }
    /**
    tf::StampedTransform transform;
    Eigen::Affine3d eig;
    tfListener.lookupTransform("/base_link", "/xtion_link",ros::Time(0),transform);
    tf::transformTFToEigen(transform, eig);
    seg.setStaticTransform(eig.cast<float>());
    */

    got_img = false;
    got_cloud = false;
    cout << topic_d << endl;
    cout << topic_rgb << endl;
    depth_sub = node.subscribe<sensor_msgs::PointCloud2>(topic_d, 1, &Communicator::depth_cb, this);
    rgb_sub = node.subscribe<sensor_msgs::Image>(topic_rgb, 1, &Communicator::rgb_cb, this);
    LOG4CXX_DEBUG(logger, "Subscribed to topics " << topic_d << ", " << topic_rgb);
    uint timeout = TIMEOUT;

    LOG4CXX_DEBUG(logger, "Waiting for cloud/rgb camera info callbacks...\n");
    while (true) {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        if (got_img && got_cloud) {
            LOG4CXX_DEBUG(logger, "Received cloud & rgb camera info callbacks\n");
            break;
        }
        if (timeout == 0) {
            LOG4CXX_WARN(logger, "Timeout reached! Was " << TIMEOUT << "\n");
            timeout = TIMEOUT;
        }
        timeout --;
    }

    vector<ImageRegion::Ptr> candidates;
    vector<Surface> tables;

    ImageSource::Ptr image(new ImageSource(image_, cloud_));
    seg.setCloud(cloud_);
    seg.setImage(image_);
    seg.segment(image, candidates, tables);
    publishResults(image, candidates, tables);
    segmentation::imageRoiResult rois = publishRoi(candidates, image);
    server.setSucceeded(rois);
    LOG4CXX_DEBUG(logger, "Results were published.\n");

}

void Communicator::publishResults(ImageSource::Ptr& image, vector<ImageRegion::Ptr>& candidates, vector<Surface>& tables) {
    LOG4CXX_DEBUG(logger, "publish results.\n");
    publishClouds(image);
    publishSupportPlanes(tables);
}

segmentation::imageRoiResult Communicator::publishRoi(vector<ImageRegion::Ptr>& candidates, ImageSource::Ptr& image){
    LOG4CXX_DEBUG(logger, "publish Regions of interest.\n");
    segmentation::imageRoiResult rois;
    for (vector<ImageRegion::Ptr>::const_iterator it = candidates.begin(); it != candidates.end(); ++it) {
        ImageRegion::Ptr item = *it;
        Roi region = item->getRoiColor();
        cv::Rect roi(region.X(), region.Y(), region.Width(), region.Height());
        cv::Mat imageROI = image_(roi);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageROI).toImageMsg();
        rois.rois.push_back(*msg);
    }
    return rois;

}

void Communicator::publishClouds(ImageSource::Ptr& image) {
    LOG4CXX_DEBUG(logger, "publish cloud.\n");
    cloud_pub.publish(image->getCloudScene());

}

void Communicator::publishSupportPlanes(vector<Surface>& tables){
    LOG4CXX_DEBUG(logger, "publish Planes.\n");

    vector<Surface>::const_iterator it;
    for (it = tables.begin(); it != tables.end(); ++it) {
        Surface item = *it;

        pcl::PointCloud<pcl::PointXYZ>::Ptr hull = item.cloudHull;
        pcl::PointNormal normal = item.cloudNormal;
        Eigen::Affine3f pose = item.getPlanePose(Eigen::Vector3f(0,0,1));
        Eigen::Vector3f translation = pose.translation();
        Eigen::Vector3f translationInv = translation * -1.0;
        Eigen::Affine3f invTransl;
        invTransl.fromPositionOrientationScale(translationInv, Eigen::AngleAxisf::Identity(), Eigen::Vector3f::Ones());
        LOG4CXX_INFO(logger, "inverse translation: " << invTransl.matrix() << "\n");

    }

}




int main(int argc, char **argv)
{
    LayoutPtr pattern(new PatternLayout("%r [%t] %-5p %c - %m&n"));
    AppenderPtr appender(new ConsoleAppender(pattern));
    Logger::getRootLogger()->addAppender(appender);
    Logger::getRootLogger()->setLevel(Level::getWarn());
    Logger::getRootLogger()->setLevel(Level::getDebug());
    ros::init(argc, argv, "segmentation");
    Segmentation seg;
    seg.setSegmentationConfig(argv[1]);
    seg.enableRctTransform(true);
    Communicator com(seg);

    ros::spin ();
    return 0;
}
