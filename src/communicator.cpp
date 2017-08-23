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
#include <sstream>
#include <limits>
#include <boost/algorithm/string.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <object_tracking_msgs/ObjectShape.h>
#include <object_tracking_msgs/DetectPlanes.h>
#include <geometry_msgs/Point.h>

using namespace std;
using namespace log4cxx;

const LoggerPtr Communicator::logger = Logger::getLogger("segmentation.communicator");

std_msgs::ColorRGBA fromRGB(int r, int g, int b)
{
    std_msgs::ColorRGBA color;
    color.a = 1;
    color.r = r / 255.0;
    color.g = g / 255.0;
    color.b = b / 255.0;
    return color;
}

std::vector<std_msgs::ColorRGBA> colors;


Communicator::Communicator(const Segmentation& segmentation): seg(segmentation) {


    segment_service = node.advertiseService("segmentation", &Communicator::get_segments, this);
    recognize_service = node.advertiseService("recognize_objects", &Communicator::recognize, this);
    planes_service = node.advertiseService("detect_planes", &Communicator::detect_planes, this);
    LOG4CXX_DEBUG(logger, "startet service Server segmentation, recognize_objects\n");

    image_path_pub = node.advertise<std_msgs::String>("image_path", 1000);
    segmented_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("segmented_object", 10);
    table_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("segmented_table", 10);

    classify_client = node.serviceClient<object_tracking_msgs::Classify>("classify");

    LOG4CXX_DEBUG(logger, "created ROS topic " << topic_pub_objects << "\n");

    // Boynton's list of 11 colors
    colors.push_back(fromRGB(0, 0, 255));      // Blue
    colors.push_back(fromRGB(255, 0, 0));      // Red
    colors.push_back(fromRGB(0, 255, 0));      // Green
    colors.push_back(fromRGB(255, 255, 0));    // Yellow
    colors.push_back(fromRGB(255, 0, 255));    // Magenta
    colors.push_back(fromRGB(255, 128, 128));  // Pink
    colors.push_back(fromRGB(128, 128, 128));  // Gray
    colors.push_back(fromRGB(128, 0, 0));      // Brown
    colors.push_back(fromRGB(255, 128, 0));    // Orange

    num_objects = 0;
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
        cv_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::cvtColor(cv_ptr->image, image_, CV_BGR2RGB);
    rgb_sub.shutdown();

    got_img = true;
}

bool Communicator::recognize(object_tracking_msgs::RecognizeObjects::Request &req, object_tracking_msgs::RecognizeObjects::Response &res)
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
    //Save Image and publish path
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
    auto str = oss.str();
    auto filename = "/home/biron/" + str + ".jpg";

    cv::imwrite(filename,image_);
    std_msgs::String msg;
    msg.data = filename;
    image_path_pub.publish(msg);

    seg.setCloud(cloud_);
    seg.setImage(image_);
    seg.segment(image, candidates, tables);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr seg_cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);


    LOG4CXX_INFO(logger, "found " << candidates.size() << " objects and " << tables.size() << " planes!\n");

    object_tracking_msgs::Classify classify;
    classify.request.objects = getRoi(candidates, image);

    // clean old object segments
    clear_segments();

    if(classify_client.call(classify)){

        // process classification results
        for (int i = 0; i < classify.response.hypotheses.size(); i++) {

            // assemble objects containing pointclouds and UUID's for fitting
            grasping_msgs::Object object;

            object.name = to_string(num_objects);
            object.support_surface = "surface0";
            sensor_msgs::PointCloud2 cloud;
            tmp = candidates[i]->getObjectCloud();
            pcl::toROSMsg(*tmp, cloud);
            object.point_cluster = cloud;
            object.header.frame_id = "base_link";

            objects.push_back(object);

            // allow all shapes until we add knowledge db
            config_names.push_back("default");

            // assemble objectlocation for response
            object_tracking_msgs::ObjectShape object_shape;

            object_shape.name = to_string(num_objects);
            // get region of interest for the Object.
            ImageRegion::Ptr item = candidates[i];
            Roi region = item->getRoiColor();
            sensor_msgs::RegionOfInterest roi;
            roi.x_offset = region.X();
            roi.y_offset = region.Y();
            roi.width = region.Width();
            roi.height = region.Height();
            object_shape.bounding_box = roi;
            object_shape.hypotheses = classify.response.hypotheses[i].hypotheses;
            geometry_msgs::Point center;
            center.x = item->getRoiCloud().xCenter();
            center.y = item->getRoiCloud().yCenter();
            center.z = item->getRoiCloud().zCenter();

            object_shape.center = center;
            object_shape.width = item->getWidth();
            object_shape.height = item->getHeight();
            object_shape.depth = item->getDepth();

            res.objects.push_back(object_shape);

            // increment object name good 'ol clafu style
            ++num_objects;


            for(pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = tmp->begin(); it!= tmp->end(); it++){
                std_msgs::ColorRGBA c = colors[i % colors.size()];
                it->r = c.r * 255;
                it->g = c.g * 255;
                it->b = c.b * 255;
            }

            *seg_cloud += *tmp;


        }

        calcSupportPlanes(tables);
        res.support_surfaces = support_planes;

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr table_cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

        int i = candidates.size();

        for( auto table : tables){
            tmp->clear();

            pcl::copyPointCloud(*table.cloud, *tmp);

            std_msgs::ColorRGBA c = colors[i++ % colors.size()];
            
            for(pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = tmp->begin(); it!= tmp->end(); it++){

                it->r = c.r * 255;
                it->g = c.g * 255;
                it->b = c.b * 255;
            }

            *table_cloud += *tmp;
        }

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*seg_cloud, cloud_msg);
        cloud_msg.header.frame_id = "base_link";
        segmented_cloud_pub.publish(cloud_msg);

        sensor_msgs::PointCloud2 table_msg;
        pcl::toROSMsg(*table_cloud, table_msg);
        table_msg.header.frame_id = "base_link";
        table_cloud_pub.publish(table_msg);

        LOG4CXX_DEBUG(logger, "Results were published.\n");
    } else {
        ROS_ERROR("Failed to call service classify");
        LOG4CXX_ERROR(logger, "Failed to call service classify.\n");
    }

    return true;
}



bool Communicator::detect_planes(object_tracking_msgs::DetectPlanes::Request &req, object_tracking_msgs::DetectPlanes::Response &res){

    LOG4CXX_DEBUG(logger, "got planes request.\n");
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

    LOG4CXX_INFO(logger, "found " << tables.size() << " planes!\n");

    calcSupportPlanes(tables);
    res.support_surfaces = support_planes;

    // clean old object segments
    clear_segments();

    return true;
}



bool Communicator::get_segments(planning_scene_manager_msgs::Segmentation::Request &req, planning_scene_manager_msgs::Segmentation::Response &res) {
    LOG4CXX_INFO(logger, "get_segments.\n");
    res.objects = objects;
    res.support_surfaces = support_planes;
    res.config_names = config_names;
    return true;
}

vector<sensor_msgs::Image> Communicator::getRoi(vector<ImageRegion::Ptr>& candidates, ImageSource::Ptr& image){
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


void Communicator::clear_segments() {
    objects.clear();
    support_planes.clear();
    config_names.clear();
}


void Communicator::calcSupportPlanes(vector<Surface>& tables){
    LOG4CXX_DEBUG(logger, "calc support Planes.\n");

    grasping_msgs::Object surfaceBig;
    geometry_msgs::PoseStamped poseBig;
    shape_msgs::SolidPrimitive primitiveBig;

    double yBig, xBig, zBig, yMaxB, yMinB;
    yBig = xBig = zBig = yMaxB = yMinB = 0;

    vector<Surface>::const_iterator it;
    for (it = tables.begin(); it != tables.end(); ++it) {
        stringstream ss;
        ss << "surface" << (it - tables.begin());
        string name = ss.str();

        Surface item = *it;
        double xMax, yMax;
        xMax = yMax = -numeric_limits<double>::max();
        double xMin, yMin;
        xMin = yMin = numeric_limits<double>::max();
        pcl::PointCloud<pcl::PointXYZ>::Ptr hull = item.cloudHull;
        pcl::PointNormal normal = item.cloudNormal;
        Eigen::Affine3f pose = item.getPlanePose(Eigen::Vector3f(0,0,1));
        Eigen::Vector3f translation = pose.translation();
        Eigen::Vector3f translationInv = translation * -1.0;
        Eigen::Affine3f invTransl;
        invTransl.fromPositionOrientationScale(translationInv, Eigen::AngleAxisf::Identity(), Eigen::Vector3f::Ones());
        LOG4CXX_INFO(logger, "inverse translation: " << invTransl.matrix() << "\n");

        //gets the z Coordinate to detect the highest and lowest plane
        double zCoord = translation[2];
        for (int i = 0; i < hull->points.size(); i++){
            pcl::PointCloud<pcl::PointXYZ>::PointType border = hull->points[i];
            if (xMax < border.x) {
                xMax = border.x;
            }
            if (yMax < border.y) {
                yMax = border.y;
            }
            if (xMin > border.x) {
                xMin = border.x;
            }
            if (yMin > border.y) {
                yMin = border.y;
            }
        }

        double xCenter = xMin + (xMax - xMin) / 2.0;
        double yCenter = yMin + (yMax - yMin) / 2.0;

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = xMax - xMin;
        primitive.dimensions[1] = yMax - yMin;
        primitive.dimensions[2] = 0.01;

        geometry_msgs::Pose poseNew;
        poseNew.position.x = translation[0];
        poseNew.position.y = translation[1];
        poseNew.position.z = translation[2];
        poseNew.orientation.w = 1;
        poseNew.orientation.x = 0;
        poseNew.orientation.y = 0;
        poseNew.orientation.z = 0;

        grasping_msgs::Object surface;
        surface.header.frame_id = "base_link";
        surface.name = ss.str();
        surface.primitive_poses.push_back(poseNew);
        surface.primitives.push_back(primitive);

        support_planes.push_back(surface);

        /**
        //the important data for the biggest surface is stored.
        if((abs(yMax - yMin)) > yBig){
            yBig = abs(yMax - yMin);
            xBig = abs(xMax - xMin);
            zBig = zCoord;
            yMaxB = yMax;
            yMinB = yMin;
            surfaceBig = surface;
            primitiveBig = primitive;
            poseBig = poseNew;
        }

        zBig = 3.00;

        //primitive is reused and parameters for left plane are used
        primitiveBig.dimensions[0] =  xBig; //length
        primitiveBig.dimensions[1] =  0.01; //depth
        primitiveBig.dimensions[2] =  zBig;//height

        moveit_msgs::CollisionObject surfaceLeft;
        surfaceLeft.header.frame_id = poseBig.header.frame_id;
        surfaceLeft.id = "surfaceBigLeft";
        surfaceLeft.operation = surfaceBig.ADD;
        surfaceLeft.primitive_poses.push_back(poseBig.pose);
        surfaceLeft.primitives.push_back(primitiveBig);
        surfaceLeft.primitive_poses[0].position.x = surfaceBig.primitive_poses[0].position.x;
        surfaceLeft.primitive_poses[0].position.y = yMinB;
        surfaceLeft.primitive_poses[0].position.z = surfaceBig.primitive_poses[0].position.z / 2;

        support_planes.push_back(surfaceLeft);

        //Transform of the right plane that is created; for comments look at leftplane
        primitiveBig.dimensions[0] =  xBig; //length
        primitiveBig.dimensions[1] =  0.01; //depth
        primitiveBig.dimensions[2] =  zBig;//height

        moveit_msgs::CollisionObject surfaceRight;
        surfaceRight.header.frame_id = poseBig.header.frame_id;
        surfaceRight.id = "surfaceBigRight";
        surfaceRight.operation = surfaceBig.ADD;
        surfaceRight.primitive_poses.push_back(poseBig.pose);
        surfaceRight.primitives.push_back(primitiveBig);
        surfaceRight.primitive_poses[0].position.x = surfaceBig.primitive_poses[0].position.x;
        surfaceRight.primitive_poses[0].position.y = yMaxB;
        surfaceRight.primitive_poses[0].position.z = surfaceBig.primitive_poses[0].position.z / 2;

        support_planes.push_back(surfaceRight);
         */
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
