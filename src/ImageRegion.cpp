/*
 * ImageRegion.cpp
 *
 *  Created on: Aug 20, 2014
 *      Author: lziegler
 */

#include "ImageRegion.h"

#include <opencv2/features2d/features2d.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/nil_generator.hpp>

using namespace std;

int64_t ImageRegion::idCounter = 0;

ImageRegion::ImageRegion() :
        id(idCounter++), imageId(boost::uuids::nil_uuid()), height(0), width(0), depth(
                0) {
}

ImageRegion::ImageRegion(uuid imageId) :
        imageId(imageId), id(idCounter++), height(0), width(0), depth(0) {
}

ImageRegion::ImageRegion(uuid imageId, int64_t regionId) :
        imageId(imageId), id(regionId), height(0), width(0), depth(0) {
    if (idCounter <= regionId)
        idCounter = regionId + 1;
}

ImageRegion::ImageRegion(const ImageRegion &other) :
        imageId(other.imageId), id(other.id), objectCloud(other.objectCloud), objectHull(
                other.objectHull), objectNormals(other.objectNormals), roiDepth(
                other.roiDepth), roiColor(other.roiColor), roiCloud(
                other.roiCloud), height(other.height), width(other.width), depth(
                other.depth), rotation(other.rotation) {

}

bool ImageRegion::hasObjectNormals() const {
    if (!objectNormals || !objectCloud) {
        return false;
    }

    if (!objectNormals->empty()
            && (objectCloud->size() == objectNormals->size())) {
        return true;
    }
    return false;
}

ImageRegion::~ImageRegion() {
}

void ImageRegion::forceIdReset() {
    id = idCounter++;
}
