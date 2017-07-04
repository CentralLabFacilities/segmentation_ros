/*
 * Results.h
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

class ResultItem {
public:
    /*
     * Center of the object
     */
    double cx, cy, cz;
    /*
     * Rotation
     */
    double qw,qx,qy,qz;
    /*
     * Width independent of rotation.
     */
    double width;
    /*
     * Height independent of rotation.
     */
    double height;
    /*
     * Depth independent of rotation.
     */
    double depth;
    std::map<std::string, float> dist;

    std::string getMostLikelyStr() const;

    float getMostLikelyVal() const;

    void normalize();
    void multiply(double factor);
    void add(const ResultItem &r);
    void add(const std::map<std::string, float> &dist);
};

class Results: public std::vector<ResultItem> {
public:
    typedef boost::shared_ptr<Results> Ptr;

    virtual ~Results() {
    }
};

namespace std {
std::ostream &operator<<(std::ostream &stream, ResultItem const &r);
}
