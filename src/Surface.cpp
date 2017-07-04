/*
 * Surfaces.h
 *
 *  Created on: May 28, 2013
 *      Author: lziegler
 */

#include "Surface.h"

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <iostream>
#include <map>
#include <string>

using namespace std;
using namespace Eigen;
using namespace log4cxx;

const Eigen::Vector3f up_vec(0,-1,0);
const Eigen::Vector3f down_vec(0,1,0);
const Eigen::Vector3f left_vec(-1,0,0);
const Eigen::Vector3f right_vec(1,0,0);
const Eigen::Vector3f front_vec(0,0,1);
const Eigen::Vector3f back_vec(0,0,-1);

Surface::Surface() :
		cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>)),
		cloudHull(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>)) {
    logger = Logger::getLogger("Surface");
}

Affine3f Surface::getPlanePose(const Vector3f &up) const {
    Vector3f normal(cloudNormal.normal_x, cloudNormal.normal_y, cloudNormal.normal_z);

    Vector4d centroid;
    pcl::compute3DCentroid(*cloudHull, centroid);

    Affine3f a = Affine3f::Identity();

    LOG4CXX_INFO(logger,"calculate pose from normal:\n" << normal);

    Affine3f pt;
    pcl::getPrincipalTransformation(*cloud, pt);

    // principal transform gives x axis as normal vector, but we want z axis to be the normal
    pt.rotate(AngleAxisf(M_PI_2, Vector3f(0,1,0)));

//    Affine3f a = Affine3f::Identity();
//    float dot = up.dot(normal);
//    if (!isnan(dot)) {
//        float angle = acos(dot);
//        LOG4CXX_INFO(logger, "angle from up to normal: " << angle);
//        if (!isnan(angle)) {
//            Vector3f cross = up.cross(normal);
//            cross.normalize();
//            LOG4CXX_INFO(logger,"rotation axis:\n" << cross);
//            AngleAxisf aa(angle, cross);
//            a.fromPositionOrientationScale(Eigen::Vector3f(centroid(0), centroid(1), centroid(2)), aa, Eigen::Vector3f::Ones());
//        }
//    }

    return pt;
}

Affine3f Surface::getPlanePoseTilt() const {

	// create mask to delete coordinates in right direction (non-tilt direction)
	Vector3f mask;
	for (int i = 0; i < 3; i++) {
		if (abs(right_vec(i)) < 0.00001)
			mask(i) = 1;
		else
			mask(i) = 0;
	}

    Vector3f normal(cloudNormal.normal_x, cloudNormal.normal_y, cloudNormal.normal_z);
    normal = normal.cwiseProduct(mask);
    normal.normalize();
    Affine3f a = Affine3f::Identity();

    float dot = normal.dot(up_vec);
    if (!isnan(dot)) {
    	// attention: the angle is absolute. unsigned!
        float angle = acos(dot);
        if (!isnan(angle)) {
        	// the cross product should be either left or right axis, depending on the
        	// rotation direction.
            Vector3f cross = normal.cross(up_vec);
            cross.normalize();
            AngleAxisf aa(angle, right_vec);
            a.fromPositionOrientationScale(Eigen::Vector3f(cloudNormal.x, cloudNormal.y, cloudNormal.z), aa, Eigen::Vector3f::Ones());
        }
    }

    return a;
}

std::ostream &std::operator<<(std::ostream &stream, Surface const &r) {
	stream << "Surface[]";
	return stream;
}

