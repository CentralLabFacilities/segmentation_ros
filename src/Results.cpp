/*
 * Results.h
 *
 *  Created on: May 28, 2013
 *      Author: lziegler
 */

#include "Results.h"
#include <iostream>
#include <map>
#include <string>
using namespace std;

std::string ResultItem::getMostLikelyStr() const {
	map<string, float>::const_iterator it;
	float max = -1;
	string maxStr = "unknown";
	for (it = dist.begin(); it != dist.end(); ++it) {
		if (max < it->second) {
			max = it->second;
			maxStr = it->first;
		}
	}
	return maxStr;
}

float ResultItem::getMostLikelyVal() const {
	map<string, float>::const_iterator it;
	float max = -1;
	for (it = dist.begin(); it != dist.end(); ++it) {
		if (max < it->second) {
			max = it->second;
		}
	}
	return max;
}

void ResultItem::normalize() {
	map<string, float>::const_iterator it;
	double sum = 0;
	for (it = dist.begin(); it != dist.end(); ++it) {
		sum += it->second;
	}
	double factor = 1 / sum;
	for (it = dist.begin(); it != dist.end(); ++it) {
		dist[it->first] *= factor;
	}

}
void ResultItem::multiply(double factor) {
	map<string, float>::iterator it;
	for (it = dist.begin(); it != dist.end(); ++it) {
		this->dist[it->first] *= factor;
	}
}
void ResultItem::add(const ResultItem &r) {
	add(r.dist);
}
void ResultItem::add(const std::map<std::string, float> &dist) {
	map<string, float>::const_iterator it;
	for (it = dist.begin(); it != dist.end(); ++it) {
		if (this->dist.count(it->first)) {
			this->dist[it->first] += it->second;
		} else {
			this->dist[it->first] = it->second;
		}
	}
}
std::ostream &std::operator<<(std::ostream &stream,
        ResultItem const &r) {
	stream << "ResultItem[cx=" << r.cx << " cy=" << r.cy << " cz=" << r.cz
			<< " width=" << r.width << " depth=" << r.depth << " height="
			<< r.height << " class:" << r.getMostLikelyStr() << "("
			<< r.getMostLikelyVal() << ")" << "]";
	return stream;
}

