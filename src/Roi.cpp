/*
 * Roi.cpp
 *
 *  Created on: Mar 21, 2011
 *      Author: lziegler, plueckin, troehlig
 */

#include "Roi.h"

using namespace std;

Roi::Roi() :
		x(0), y(0), width(0), height(0) {
}

Roi::Roi(int x, int y, int width, int height) :
		x(x), y(y), width(width), height(height) {
}

Roi::~Roi() {
}

bool Roi::save(cv::FileStorage fs, const string &suffix) {
	fs << "roi_"+suffix;
	fs << "{:";
	fs << "x" << x;
	fs << "y" << y;
	fs << "width" << width;
	fs << "height" << height;
	fs << "}";
	return true;
}
bool Roi::load(cv::FileStorage fs, const string &suffix) {
	cv::FileNode n;
	n = fs[string("roi_")+suffix];
	n["x"] >> x;
	n["y"] >> y;
	n["width"] >> width;
	n["height"] >> height;
	return true;
}
bool Roi3D::save(cv::FileStorage fs, const string &suffix) {
    fs << "roi3d" << suffix << "{:";
    fs << "x" << xMin;
    fs << "y" << yMin;
    fs << "z" << zMin;
    fs << "width" << width;
    fs << "height" << height;
    fs << "depth" << depth;
    fs << "}";
    return true;
}
bool Roi3D::load(cv::FileStorage fs, const string &suffix) {
    cv::FileNode n;
    n = fs[string("roi3d")+suffix];
    n["x"] >> xMin;
    n["y"] >> yMin;
    n["z"] >> zMin;
    n["width"] >> width;
    n["height"] >> height;
    n["depth"] >> depth;
    return true;
}
std::ostream &operator<<(std::ostream &stream, Roi const &r) {
	stream << r.x << " " << r.y << " " << r.width << " " << r.height;
	return stream;
}

std::istream &operator>>(std::istream &stream, Roi &r) {
	stream >> r.x >> r.y >> r.width >> r.height;
	return stream;
}
