/** @brief Represents the region of interest.
 *
 * Used for narrowing the resolution of video and depth device.
 @author lziegler, troehlig
 @date March 2011
 */

#pragma once

#include <iostream>
#include <opencv2/core/core.hpp>
#include <math.h>

class Roi {
public:
    Roi();
    Roi(int x, int y, int width, int height);

    virtual ~Roi();

    inline Roi zoom(float const sh, float const sw) {
        x = x * sw;
        y = y * sh;
        width = width * sw;
        height = height * sh;
        return *this;
    }

    inline Roi scaleInPlace(float const sh, float const sw, int imgWidth, int imgHeight) {
        int oldWidth = width;
        int oldHeight = height;
        width = int((float)oldWidth * sw);
        height = int((float)oldHeight * sh);
        x = x + (oldWidth - width) / 2.0;
        y = y + (oldHeight - height) / 2.0;
        if (x < 0) x = 0;
        if (y < 0) y = 0;
        if (x+width>imgWidth) width -= (x+width-imgWidth);
        if (y+height>imgHeight) height -= (y+height - imgHeight);
        return *this;
    }

    inline Roi scaleAbsolute(float const factor) {
		int oldWidth = width;
		int oldHeight = height;
		width = int((float)oldWidth * factor);
		height = int((float)oldHeight * factor);
		x = x * factor;
		y = y * factor;
		return *this;
	}

    Roi &operator=(Roi const &other) {
        x = other.x;
        y = other.y;
        width = other.width;
        height = other.height;
        return *this;
    }

    inline int Height() const {
        return height;
    }

    inline int Width() const {
        return width;
    }

    inline int X() const {
        return x;
    }

    inline int Y() const {
        return y;
    }

    inline int bottom() const {
        return y + height;
    }

    inline int right() const {
        return x + width;
    }

    inline int left() const {
        return x;
    }

    inline int top() const {
        return y;
    }

    bool save(cv::FileStorage fs, const std::string &suffix);
    bool load(cv::FileStorage fs, const std::string &suffix);
    int x;
    int y;
    int width;
    int height;
};

class Roi3D {
public:
    Roi3D() :
            xMin(0), yMin(0), zMin(0), width(0), depth(0), height(0) {

    }
    Roi3D(double xMin, double yMin, double zMin, double width, double height, double depth) :
            xMin(xMin), yMin(yMin), zMin(zMin), width(width), depth(depth), height(height) {
    }

    virtual ~Roi3D() {
    }

    inline Roi3D zoom(float const sh, float const sw, float const sd) {
        xMin = xMin * sw;
        yMin = yMin * sh;
        zMin = zMin * sd;
        width = width * sw;
        height = height * sh;
        depth = depth * sd;
        return *this;
    }

    inline Roi3D scale(float const sh, float const sw, float const sd) {
        double oldWidth = width;
        double oldHeight = height;
        double oldDepth = depth;
        width = oldWidth * sw;
        height = oldHeight * sh;
        depth = oldDepth * sh;
        xMin = xMin + (oldWidth - width) / 2;
        yMin = yMin + (oldHeight - height) / 2;
        zMin = zMin + (oldDepth - depth) / 2;
        return *this;
    }

    Roi3D &operator=(Roi3D const &other) {
        xMin = other.xMin;
        yMin = other.yMin;
        zMin = other.zMin;
        width = other.width;
        height = other.height;
        depth = other.depth;
        return *this;
    }

    /*
     * size in y direction
     */
    inline double Height() const {
        return height;
    }

    /*
     * size in x direction
     */
    inline double Width() const {
        return width;
    }

    /*
     * size in z direction
     */
    inline double Depth() const {
        return depth;
    }

    inline double XMin() const {
        return xMin;
    }

    inline double YMin() const {
        return yMin;
    }

    inline double ZMin() const {
        return zMin;
    }

    inline double xCenter() const {
        return xMin + (width / 2.0);
    }

    inline double yCenter() const {
        return yMin + (height / 2.0);
    }

    inline double zCenter() const {
        return zMin + (depth / 2.0);
    }
    inline double bottom() const {
        return yMin + height;
    }

    inline double right() const {
        return xMin + width;
    }

    inline double left() const {
        return xMin;
    }

    inline double top() const {
        return yMin;
    }

    bool save(cv::FileStorage fs, const std::string &suffix);
    bool load(cv::FileStorage fs, const std::string &suffix);
    double xMin;
    double yMin;
    double zMin;

    /*
     * size in x direction
     */
    double width;
    /*
     * size in z direction
     */
    double depth;
    /*
     * size in y direction
     */
    double height;

    inline const bool isValid() {
        bool nan = std::isnan(xMin) || std::isnan(yMin) || std::isnan(zMin) || std::isnan(width) || std::isnan(depth) || std::isnan(height);
        bool inf = std::isinf(xMin) || std::isinf(yMin) || std::isinf(zMin) || std::isinf(width) || std::isinf(depth) || std::isinf(height);
        return !nan && !inf;
    }

};

std::ostream &operator<<(std::ostream &stream, Roi const &r);

std::istream &operator>>(std::istream &stream, Roi &r);

