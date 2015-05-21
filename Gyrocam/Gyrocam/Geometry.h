#pragma once
#include "opencv2/core/core.hpp"

using namespace cv;
using namespace std;

namespace gyrocam
{
	Point3f lineThroughPoints(Point3f p1, Point3f p2);
	Point3f intersection(Point3f l1, Point3f l2);
	double incidence(Point3f p1, Point3f p2);
	bool isInfinitePoint(Point3f p);
	void normalizeZ(Point3f &p);
}