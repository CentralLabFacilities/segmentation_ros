/*
 * Surfaces.h
 *
 *  Created on: May 28, 2013
 *      Author: lziegler
 */

#pragma once

#include <string>
#include <map>
#include <vector>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <log4cxx/logger.h>

#include <Eigen/Geometry>

class Surface {
private:
    log4cxx::LoggerPtr logger;
public:

    Surface();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudHull;
    pcl::PointNormal cloudNormal;

    Eigen::Affine3f getPlanePose(const Eigen::Vector3f &up) const;
    Eigen::Affine3f getPlanePoseTilt() const;
};

namespace std {
std::ostream &operator<<(std::ostream &stream, Surface const &r);
}
