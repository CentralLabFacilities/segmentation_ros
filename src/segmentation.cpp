#include <fstream>
#include "segmentation.h"
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/any.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <string.h>

using namespace std;
using namespace log4cxx;

const LoggerPtr Segmentation::logger = Logger::getLogger("segmentation.segmentation");
namespace po = boost::program_options;

Segmentation::Segmentation(){
    prepro.reset(new pca::Preprocessor(false, true, false, true, false, false));
    prepro->initOptions(processorOptions);
    //pcaplugin.reset(new pca::MultiPlaneObjectProcessor());
    setProcessor();
}

void Segmentation::PrintVariableMap(const boost::program_options::variables_map& vm) {
    for (const auto& it: vm) {
        std::cout << "> " << it.first;
        if (((boost::any) it.second.value()).empty()) {
            std::cout << "(empty)";
        }
        if (vm[it.first].defaulted() || it.second.defaulted()) {
            std::cout << "(default)";
        }
        std::cout << "=";

        bool is_char;
        try {
            boost::any_cast<const char *>(it.second.value());
            is_char = true;
        } catch (const boost::bad_any_cast &) {
            is_char = false;
        }
        bool is_str;
        try {
            boost::any_cast<std::string>(it.second.value());
            is_str = true;
        } catch (const boost::bad_any_cast &) {
            is_str = false;
        }

        if (((boost::any) it.second.value()).type() == typeid(int)) {
            std::cout << vm[it.first].as<int>() << std::endl;
        } else if (((boost::any) it.second.value()).type() == typeid(bool)) {
            std::cout << vm[it.first].as<bool>() << std::endl;
        } else if (((boost::any) it.second.value()).type() == typeid(double)) {
            std::cout << vm[it.first].as<double>() << std::endl;
        } else if (is_char) {
            std::cout << vm[it.first].as<const char *>() << std::endl;
        } else if (is_str) {
            std::string temp = vm[it.first].as<std::string>();
            if (temp.size()) {
                std::cout << temp << std::endl;
            } else {
                std::cout << "true" << std::endl;
            }
        } else { // Assumes that the only remainder is vector<string>
            try {
                std::vector<std::string> vect = vm[it.first].as<std::vector<std::string> >();
                uint i = 0;
                for (std::vector<std::string>::iterator oit = vect.begin();
                     oit != vect.end(); oit++, ++i) {
                    std::cout << "\r> " << it.first << "[" << i << "]=" << (*oit) << std::endl;
                }
            } catch (const boost::bad_any_cast &) {
                std::cout << "UnknownType(" << ((boost::any) it.second.value()).type().name() << ")" << std::endl;
            }
        }
    }
}

void Segmentation::setProcessor() {

    if (multiPlane) {
        pcaplugin.reset(new pca::MultiPlaneObjectProcessor());
    } else {
        pcaplugin.reset(new pca::ObjectProcessor());
    }
    pcaplugin->initOptions(processorOptions);

    boost::program_options::store(boost::program_options::parse_environment(processorOptions, "PCA"), parameters);
    PrintVariableMap(parameters);

}

void Segmentation::setCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
    cloud_ = cloud;
}

void Segmentation::setImage(cv::Mat image) {
    image_ = image;
}

void Segmentation::setSegmentationConfig(string const &path) {
    ifstream ifs(path.c_str());
    LOG4CXX_DEBUG(logger, "PCA config file: " << path << "\n");

    pcaplugin->initOptions(processorOptions);
    boost::program_options::store(boost::program_options::parse_config_file(ifs, processorOptions, true), parameters);
    boost::program_options::notify(parameters);
    PrintVariableMap(parameters);
}

void Segmentation::setStaticTransform(Eigen::Affine3f const &transform) {
    prepro->setTransformationMethod(pca::STATIC_TRANSFORM);
    prepro->setStaticTransform(transform);
}


void Segmentation::enableRctTransform(bool useRct) {
    if (useRct) {
        prepro->setTransformationMethod(pca::RCT_TRANSFORM);
    } else {
        prepro->setTransformationMethod(pca::STATIC_TRANSFORM);
    }
}

void Segmentation::segment(ImageSource::Ptr image, vector<ImageRegion::Ptr>& regionsOut, vector<Surface>& tablesOut) {
    vector<pca::CloudNHull<pcl::PointNormal> > clusters;
    int imgWidth = image->getImageColor().cols;
    int imgHeight = image->getImageColor().rows;

    try {
        pcl::PointCloud<pcl::PointNormal>::Ptr pointCloud(new pcl::PointCloud<pcl::PointNormal>());
        pcl::IndicesPtr indices(new vector<int>);
        pcl::copyPointCloud(*image->getCloudScene(), *pointCloud);


        LOG4CXX_DEBUG(logger, "Segmenting! image / cloud size: " << imgWidth << "x" << imgHeight << " / " << pointCloud->width << "x" << pointCloud->height << "\n");

        for (uint i = 0; i < pointCloud->size(); i++) {
            pcl::PointNormal p = pointCloud->at(i);
            if (p.x != .0 && p.y != .0 && p.z != .0)
                indices->push_back(i);
        }

        pca::CloudAndIndices<PointNormal> cloud(pointCloud, indices);

        LOG4CXX_DEBUG(logger, "preprocess...\n");
        prepro->process(cloud, parameters);

        LOG4CXX_DEBUG(logger, "process...\n");
        pcaplugin->setData(cloud, parameters);
        pcaplugin->process(clusters);

        if (multiPlane) {
            LOG4CXX_DEBUG(logger, "multiplane processing...\n");
            vector<pair<pcl::PointNormal, pcl::PointCloud<pcl::PointNormal>>> lastPlanes;
            ((pca::MultiPlaneObjectProcessor*)pcaplugin.get())->getLastPlanes(lastPlanes);
            LOG4CXX_DEBUG(logger, "# of planes: " << lastPlanes.size() << "\n");
            for (uint i = 0; i < lastPlanes.size(); i++) {
                pcl::PointNormal tableNormal = lastPlanes[i].first;
                pcl::PointCloud<pcl::PointNormal> tableCloud = lastPlanes[i].second;
                pca::CloudNHull<PointNormal> tableCNH(boost::make_shared<pcl::PointCloud<pcl::PointNormal> >(tableCloud));

                Surface surface;
                surface.cloudNormal = tableNormal;
                pcl::copyPointCloud(*tableCNH.getInliers().cloud, *surface.cloud);
                pcl::copyPointCloud(*tableCNH.getHull(), *surface.cloudHull);

                tablesOut.push_back(surface);
            }
        } else {

            pcl::PointCloud<pcl::PointNormal>::Ptr table(new pcl::PointCloud<pcl::PointNormal>());
            pcl::PointNormal planeVector;
            ((pca::ObjectProcessor*)pcaplugin.get())->getLastPlane(planeVector, *table);

            pca::CloudNHull<pcl::PointNormal> tableCNH(table);

            Surface surface;
            surface.cloudNormal = planeVector;
            pcl::copyPointCloud(*tableCNH.getInliers().cloud, *surface.cloud);
            pcl::copyPointCloud(*tableCNH.getHull(), *surface.cloudHull);

            tablesOut.push_back(surface);
        }

        //create and fill object
        for (uint i = 0; i < clusters.size(); i++) {
            LOG4CXX_DEBUG(logger, "cluster processing...\n");

            pca::CloudNHull<pcl::PointNormal> coudNhull = clusters.at(i);

            if (coudNhull.getType() == pca::SURFACE) {
                continue;
            }

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::copyPointCloud(*coudNhull.getInliers().cloud, *coudNhull.getInliers().indices, *objCloud);

            LOG4CXX_DEBUG(logger, "pca shape - w:" << coudNhull.getSizeY() << " h:" << coudNhull.getSizeZ() << " d:" << coudNhull.getSizeX() << "\n");

            float distanceThreshold = 0;
            try {
                if (multiPlane) {
                    distanceThreshold = parameters["multiplaneobjectprocessor.distance_threshold"].as<float>();
                } else {
                    distanceThreshold = parameters["objectprocessor.distance_threshold"].as<float>();
                }
            } catch (boost::exception &e) {
                LOG4CXX_ERROR(logger, "cannot read parameter multiplaneobjectprocessor.distance_threshold\n");
            }
            LOG4CXX_DEBUG(logger, "add distance threshold to height: " << distanceThreshold << "\n");

            pca::BoundingBox box = coudNhull.getBoundingBox();

            // refactoring new

            ImageRegion::Ptr region(new ImageRegion(image->getUid()));
            region->setRoiDepth(Roi(box.xMin, box.yMin, (box.xMax - box.xMin), (box.yMax - box.yMin)));

            Roi roi;

            tfDepthToColor(region->getRoiDepth(), roi);
            /**
            roi.scaleInPlace(bbScale, bbScale, imgWidth, imgHeight);
            if (grabber->getRgbInputSource() == GPHOTO2) {
                roi.scaleAbsolute(imgScale);
            }
            */
            region->setRoiColor(roi);
            LOG4CXX_DEBUG(logger, "roi depth (x;y;width;height): " << region->getRoiDepth().x << ";" << region->getRoiDepth().y << ";" << region->getRoiDepth().width << ";" << region->getRoiDepth().height << "\n");
            LOG4CXX_DEBUG(logger, "roi color (x;y;width;height): " << region->getRoiColor().x << ";" << region->getRoiColor().y << ";" << region->getRoiColor().width << ";" << region->getRoiColor().height << "\n");

            Roi3D roi3d = clusterToRoi3D(coudNhull);
            LOG4CXX_DEBUG(logger, "roi 3D: " << roi3d.xMin << ";" << roi3d.yMin << ";" << roi3d.zMin << "   "  << roi3d.width << ";" << roi3d.height << ";" << roi3d.depth << "\n");
            region->setRoiCloud(roi3d);
            region->setDepth(coudNhull.getSizeX());
            region->setWidth(coudNhull.getSizeY());
            region->setHeight(coudNhull.getSizeZ() + distanceThreshold);
            region->setObjectCloud(objCloud);

            regionsOut.push_back(region);
        }
    } catch (pca::AnalyzerException& e) {
        LOG4CXX_ERROR(logger, "PCA error: " << e.what() << "\n");
    }

}

void Segmentation::tfDepthToColor(const Roi& in, Roi& out) {
    int sourceWidth = DEFAULT_RGB_WIDTH;
    int sourceHeight = DEFAULT_RGB_HEIGHT;

    // Find view dimensions
    int width = DEFAULT_DEPTH_WIDTH;
    int height = DEFAULT_DEPTH_HEIGHT;

    float wScale = (float)width/(float)sourceWidth;
    float hScale = (float)height/(float)sourceHeight;
    float scale = min(wScale, hScale);

    int offsetX = 0;
    int offsetY = 0;
    if (wScale <= hScale) {
        offsetY = (int)((height/2) - ((scale * sourceHeight)/2));
    } else {
        offsetX = (int)((width/2) - ((scale * sourceWidth)/2));
    }

    // Convert depth coordinates to image coordinates
    out = in;
    out.x = (int)((in.x - offsetX) / scale);
    out.y = (int)((in.y - offsetY) / scale);
    out.width = in.width / wScale;
    out.height = in.height / hScale;
}
Roi3D Segmentation::clusterToRoi3D(pca::CloudNHull<PointNormal> &cloud) const {
    Roi3D roi;
    Eigen::Vector4f min, max;
    pcl::getMinMax3D(*cloud.getInliers().cloud, *cloud.getInliers().indices, min, max);
    roi.xMin = min[0];
    roi.yMin = min[1];
    roi.zMin = min[2];
    roi.width = max[0] - min[0];
    roi.height = max[1] - min[1];
    roi.depth = max[2] - min[2];

    return roi;
}

