#pragma once
#include "opencv2/core/core.hpp"

using namespace cv;
using namespace std;

namespace gyrocam
{
	Point3d lineThroughPoints(Point3d p1, Point3d p2);
	Point3d intersection(Point3d l1, Point3d l2);
	double incidence(Point3d p1, Point3d p2);
	bool isInfinitePoint(Point3d p);
	void normalizeZ(Point3d &p);
}