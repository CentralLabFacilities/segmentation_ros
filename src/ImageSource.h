/*
 * ImageSource.h
 *
 *  Created on: Aug 19, 2014
 *      Author: lziegler
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include <boost/uuid/uuid.hpp>

typedef boost::uuids::uuid uuid;

class ImageSource {
public:
	typedef boost::shared_ptr<ImageSource> Ptr;
	ImageSource();
	ImageSource(uuid uid);
	ImageSource(const std::string& imagePath);
    ImageSource(const cv::Mat& imageColor);
	ImageSource(const cv::Mat& imageColor, const cv::Mat& imageDepth, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloudScene);
    ImageSource(const cv::Mat& imageColor, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloudScene);
	ImageSource(uuid uid, const cv::Mat& imageColor);
	ImageSource(uuid uid, const cv::Mat& imageDepth, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloudScene);
	ImageSource(uuid uid, const cv::Mat& imageColor, const cv::Mat& imageDepth, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloudScene);

	uuid getUid() const;

	virtual ~ImageSource();

	virtual bool hasData(){
		return hasDepthData() || hasColorData() || hasSceneCloud();
	}
	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& getCloudScene() const {
		return cloudScene;
	}

	void setCloudScene(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloudScene) {
		this->cloudScene = cloudScene;
	}

	const cv::Mat& getImageColor() const {
		return imageColor;
	}

	const std::string getPath() const {
		return path;
	}

	void setImageColor(const cv::Mat& imageColor) {
		this->imageColor = imageColor;
	}

	const cv::Mat& getImageDepth() const {
		return imageDepth;
	}

	void setImageDepth(const cv::Mat& imageDepth) {
		this->imageDepth = imageDepth;
	}

	bool hasSceneCloud() {
        return cloudScene != NULL;
    }

	bool hasDepthData() {
		return this->imageDepth.cols > 0;
	}

	bool hasColorData() {
		return this->imageColor.cols > 0;
	}

	bool hasPath() {
		return this->path != "";
	}

	bool isCloudRegistered() const {
		return cloudRegistered;
	}

	void setCloudRegistered(bool registered) {
		cloudRegistered = registered;
	}

private:

	uuid uid;
	bool cloudRegistered;
	std::string path;

	/**
	 * Point cloud of the scene
	 */
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudScene;

	/**
	 * Pointer to rgb image data
	 */
	cv::Mat imageColor;

	/**
	 * Pointer to depth image data
	 */
	cv::Mat imageDepth;

};
