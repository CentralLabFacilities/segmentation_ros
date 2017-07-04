/*
 * ImageSource.cpp
 *
 *  Created on: Aug 19, 2014
 *      Author: lziegler
 */

#include "ImageSource.h"

#include <boost/uuid/random_generator.hpp>

ImageSource::ImageSource() :
		uid(boost::uuids::random_generator()()),
		cloudRegistered(false) {
}
ImageSource::ImageSource(uuid uid) :
		uid(uid),
		cloudRegistered(false) {
}

ImageSource::ImageSource(
		const cv::Mat& imageColor,
		const cv::Mat& imageDepth,
		const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloudScene) :
        uid(boost::uuids::random_generator()()),
		cloudScene(cloudScene),
		cloudRegistered(true) {
	imageColor.copyTo(this->imageColor);
	imageDepth.copyTo(this->imageDepth);

}

ImageSource::ImageSource(const cv::Mat& imageColor) :
        uid(boost::uuids::random_generator()()),
		cloudRegistered(true) {
	imageColor.copyTo(this->imageColor);
}

ImageSource::ImageSource(const std::string& imagePath) :
        uid(boost::uuids::random_generator()()),
		path(imagePath),
		cloudRegistered(true) {
}

ImageSource::ImageSource(const cv::Mat& imageColor, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloudScene) :
        uid(boost::uuids::random_generator()()),
        cloudScene(cloudScene),
        cloudRegistered(true) {
    imageColor.copyTo(this->imageColor);
}

ImageSource::ImageSource(
		uuid id,
		const cv::Mat& imageColor,
		const cv::Mat& imageDepth,
		const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloudScene) :
        uid(id),
		cloudScene(cloudScene),
		cloudRegistered(true) {
	imageColor.copyTo(this->imageColor);
	imageDepth.copyTo(this->imageDepth);
}

ImageSource::ImageSource(uuid id,const cv::Mat& imageColor) :
        uid(id),
		imageColor(imageColor),
		cloudRegistered(true) {

}

ImageSource::ImageSource(uuid id, const cv::Mat& imageDepth, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloudScene) :
        uid(id),
		imageDepth(imageDepth),
		cloudScene(cloudScene),
		cloudRegistered(true) {
	imageDepth.copyTo(this->imageDepth);

}

uuid ImageSource::getUid() const {
    return uid;
}

ImageSource::~ImageSource() {
}
