#pragma once
#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "Surface.h"
#include "ImageRegion.h"
#include "ImageSource.h"
#include <boost/program_options.hpp>
#include <log4cxx/logger.h>
#include <pca/analyzing/plugins/MultiPlaneObjectProcessor.h>
#include <pca/analyzing/Plugin.h>
#include <pca/analyzing/PreProcessor.h>
#include <pca/analyzing/plugins/ObjectProcessor.h>
#include <pca/analyzing/plugins/MultiPlaneObjectProcessor.h>
#include "Roi.h"
#include <Eigen/Geometry>


class Segmentation {
private:
    boost::shared_ptr<pca::Preprocessor> prepro;
    boost::shared_ptr<pca::Plugin> pcaplugin;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
    cv::Mat image_;

    boost::program_options::options_description processorOptions;
    boost::program_options::variables_map parameters;
    void PrintVariableMap(const boost::program_options::variables_map& vm);
    void setProcessor();
    bool multiPlane = true;


    const int DEFAULT_DEPTH_WIDTH = 640;
    const int DEFAULT_DEPTH_HEIGHT = 480;
    const int DEFAULT_RGB_WIDTH = 640;
    const int DEFAULT_RGB_HEIGHT = 480;

    static const log4cxx::LoggerPtr logger;
public:
    Segmentation();
    void segment(ImageSource::Ptr image, vector<ImageRegion::Ptr>& regionsOut, vector<Surface>& tablesOut);
    void tfDepthToColor(const Roi& in, Roi& out);
    Roi3D clusterToRoi3D(pca::CloudNHull<PointNormal> &cloud) const;
    void enableRctTransform(bool useRct);
    void setSegmentationConfig(string const &path);
    void setCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
    void setImage(cv::Mat image);
    void setStaticTransform(Eigen::Affine3f const &transform);
};

#endif
