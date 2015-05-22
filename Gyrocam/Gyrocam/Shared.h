#pragma once
#include<set>

#include "opencv2/core/core.hpp"
#include "LineSegment.h"

using namespace cv;
using namespace std;

namespace gyrocam
{
	Point3d toProjective(Vec4i segment, bool first = true);
	Point3d toProjective(double x, double y);
	double norm12(Point3d p);
	bool isIncident(Point3d vp, LineSegment segment);
	void setRow(Mat a, int i, Mat col);
	
	int iterateOnSet(set<int> &base, set<int>::iterator &it, int shift);
}