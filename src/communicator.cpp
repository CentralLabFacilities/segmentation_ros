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
#include <object_recognition_msgs/segcla.h>

using namespace std;
using namespace log4cxx;

const LoggerPtr Communicator::logger = Logger::getLogger("segmentation.communicator");

//Communicator::Communicator(const Segmentation& segmentation): seg(segmentation), server(node, "segment", boost::bind(&Communicator::segment_cb, this, _1), false){
Communicator::Communicator(const Segmentation& segmentation): seg(segmentation) {

    //segment_sub = node.subscribe("segment", 10, &Communicator::segment_cb, this);
    LOG4CXX_DEBUG(logger, "subscribed to Topic segment\n");

    planning_service = node.advertiseService("getSegments", &Communicator::getSegments, this);
    segment_service = node.advertiseService("segment", &Communicator::segment_cb, this);


    class_client = node.serviceClient<object_recognition_msgs::segcla>("classify");

    //object_pub = node.advertise<grasping_msgs::GraspObjects>(topic_pub_objects, 10);
    LOG4CXX_DEBUG(logger, "created ROS topic " << topic_pub_objects << "\n");
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

bool Communicator::segment_cb(object_recognition_msgs::segment::Request &req, object_recognition_msgs::segment::Response &res)
{
    LOG4CXX_DEBUG(logger, "got segmentation request.\n");
    bool d = is_published(topic_d);
    bool rgb = is_published(topic_rgb);
    if (!d || !rgb) {
        LOG4CXX_WARN(logger, "neither " << topic_d << ", nor " << topic_rgb << " are available!\n");
    }

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
    object_recognition_msgs::segcla rois;
    rois.request.objects = publishRoi(candidates, image);

    if(class_client.call(rois)){
        std::vector<std::string> labels = rois.response.labels;
        res.labels = labels;
        for (int i = 0; i < labels.size(); i++) {
            grasping_msgs::Object object;
            object.name = labels[i];
            object.support_surface = "plane_0";
            sensor_msgs::PointCloud2 cloud;
            pcl::toROSMsg(*(candidates[i]->getObjectCloud()), cloud);
            object.point_cluster = cloud;

            objects.push_back(object);
        }

        for (int i = 0; i < tables.size(); i++){
            grasping_msgs::Object plane;
            plane.name = "plane_0";

            support_planes.push_back(plane);
        }

        LOG4CXX_DEBUG(logger, "Results were published.\n");
    } else {
        ROS_ERROR("Failed to call service classify");
        LOG4CXX_ERROR(logger, "Failed to call service classify.\n");
    }

    return true;
}

bool Communicator::getSegments(planning_scene_manager_msgs::Segmentation::Request &req, planning_scene_manager_msgs::Segmentation::Response &res) {
    res.objects = objects;
    res.support_surfaces = support_planes;
    res.config = configs;
    return true;
}

void Communicator::publishResults(ImageSource::Ptr& image, vector<ImageRegion::Ptr>& candidates, vector<Surface>& tables) {
    LOG4CXX_DEBUG(logger, "publish results.\n");
    publishClouds(image);
    //publishSupportPlanes(tables);
}

vector<sensor_msgs::Image> Communicator::publishRoi(vector<ImageRegion::Ptr>& candidates, ImageSource::Ptr& image){
    LOG4CXX_DEBUG(logger, "publish Regions of interest.\n");
    vector<sensor_msgs::Image> rois;
    for (vector<ImageRegion::Ptr>::const_iterator it = candidates.begin(); it != candidates.end(); ++it) {
        ImageRegion::Ptr item = *it;
        Roi region = item->getRoiColor();
        cv::Rect roi(region.X(), region.Y(), region.Width(), region.Height());
        cv::Mat imageROI = image_(roi);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageROI).toImageMsg();
        rois.push_back(*msg);
    }
    return rois;

}

void Communicator::publishClouds(ImageSource::Ptr& image) {
    LOG4CXX_DEBUG(logger, "publish cloud.\n");
    //cloud_pub.publish(image->getCloudScene());

}
/**
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

        segmentation::PolygonialPath3D patch;
        patch.base.position.x(translation[0]);
        patch.base.position.y(translation[1]);
        patch.base.position.z(translation[2]);
        patch.base.position.frame_id("base_link");
        patch.base.rotation.x(0);
        patch.base.rotation.y(0);
        patch.base.rotation.z(0);
        patch.base.rotation.w(1);

        for (int i = 0; i < hull->points.size(); i++){
            pcl::PointCloud<pcl::PointXYZ>::PointType p = hull->points[i];
            geometry_msgs::Point border;
            border.x(p.x);
            border.y(p.y);
            patch.border.push_back(border);
            LOG4CXX_INFO(logger, string("patch border: ") << p.x << "," << p.y << "(,"<< p.z<<")");
        }
        plane_pub.publish(patch);
    }

}
*/



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
