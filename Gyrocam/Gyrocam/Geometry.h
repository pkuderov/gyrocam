#pragma once
#include "opencv2/core/core.hpp"

namespace gyrocam
{
	cv::Point3d toProjective(double x, double y);
	cv::Point3d lineThroughPoints(const cv::Point3d &p1, const cv::Point3d &p2);
	cv::Point3d intersection(const cv::Point3d &l1, const cv::Point3d &l2);
	double incidence(const cv::Point3d &p1, const cv::Point3d &p2);
	bool isInfinitePoint(const cv::Point3d &p);
	double norm12(const cv::Point3d &p);

	void normalizeZ(cv::Point3d &p);
}