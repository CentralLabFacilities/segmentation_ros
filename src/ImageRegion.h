/*
 * ImageRegion.h
 *
 *  Created on: Aug 20, 2014
 *      Author: lziegler
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Roi.h"
#include <Eigen/Geometry>
#ifndef Q_MOC_RUN
#include <boost/uuid/uuid.hpp>
#endif

//forward declarations
namespace cv {
    class KeyPoint;
}

//

typedef boost::uuids::uuid uuid;

class ImageRegion {
public:
	typedef boost::shared_ptr<ImageRegion> Ptr;

	ImageRegion();
	ImageRegion(uuid imageId);
	ImageRegion(uuid imageId, int64_t regionId);
	ImageRegion(const ImageRegion &other);
	virtual ~ImageRegion();

	double getDepth() const {
		return depth;
	}

	void setDepth(double depth) {
		this->depth = depth;
	}

	double getHeight() const {
		return height;
	}

	void setHeight(double height) {
		this->height = height;
	}

	uuid getImageId() const {
		return imageId;
	}

	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& getObjectCloud() const {
		return objectCloud;
	}

	void setObjectCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& objectCloud) {
		this->objectCloud = objectCloud;
	}

	const pcl::PointCloud<pcl::PointXYZ>::Ptr& getObjectHull() const {
		return objectHull;
	}

	void setObjectHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr& objectHull) {
		this->objectHull = objectHull;
	}

	const pcl::PointCloud<pcl::Normal>::Ptr& getObjectNormals() const {
		return objectNormals;
	}

	void setObjectNormals(const pcl::PointCloud<pcl::Normal>::Ptr& objectNormals) {
		this->objectNormals = objectNormals;
	}

	const Roi3D& getRoiCloud() const {
		return roiCloud;
	}

	void setRoiCloud(const Roi3D& roiCloud) {
		this->roiCloud = roiCloud;
	}

	const Roi& getRoiColor() const {
		return roiColor;
	}

	void setRoiColor(const Roi& roiColor) {
		this->roiColor = roiColor;
	}

	const Roi& getRoiDepth() const {
		return roiDepth;
	}

	void setRoiDepth(const Roi& roiDepth) {
		this->roiDepth = roiDepth;
	}

	const Eigen::Quaternionf& getRotation() const {
		return rotation;
	}

	void setRotation(const Eigen::Quaternionf& rotation) {
		this->rotation = rotation;
	}

	double getWidth() const {
		return width;
	}

	void setWidth(double width) {
		this->width = width;
	}

	int64_t getId() const {
		return id;
	}

	/**
	  * Checks whether the normal for the point cloud in the object pointer
	  * needs to be recomputed (size of normal cloud is unequal to the size
	  * of point cloud)
	  * @return false if the size of normal cloud and unequal to the size of point cloud and both are not empty, true otherwise
	  */
	bool hasObjectNormals() const;

	bool hasCloud() const {
	    return objectCloud != NULL;
	}
	bool hasHull() const {
        return objectHull != NULL;
    }
	bool hasImage() const {
        return !imageId.is_nil();
    }

protected:
	void forceIdReset();

private:

	uuid imageId;
	/**
	 * Point cloud representing this object
	 */
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objectCloud;
	/**
	 * Object hull containing this object
	 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr objectHull;
	/**
	 * Normals of the object points in point cloud
	 */
	pcl::PointCloud<pcl::Normal>::Ptr objectNormals;
	/**
	 * Roi representing the 2D depth position
	 */
	Roi roiDepth;
	/**
	 * Roi representing the 2D image position
	 */
	Roi roiColor;
	/**
	 * Roi representing the 3D position
	 */
	Roi3D roiCloud;
    /**
     * These are size parameters, that are independent of the camera's viewpoint.
     */
    double width, height, depth;

    /**
     * Rotation for reconstructing real bounding box
     */
    Eigen::Quaternionf rotation;

	int64_t id;

    static int64_t idCounter;
};
