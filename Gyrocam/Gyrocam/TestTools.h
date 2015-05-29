#pragma once
#include "opencv2/core/core.hpp"
#include <vector>
#include "LineSegment.h"

using namespace std;
using namespace cv;

namespace gyrocam
{
	void testOut(Vec4i s);
	void testOut(Point3d p);

	vector<Scalar> getColors();
	void drawFoundSegments(vector<LineSegment> segments, Mat image, Scalar color, int thickness = 2);
	void drawPointMarkerIfVisibleOnImage(Point3d p, Mat image, Scalar color);
}

