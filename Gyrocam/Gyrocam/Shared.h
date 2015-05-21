#pragma once
#include<set>

#include "opencv2/core/core.hpp"
#include "LineSegment.h"

using namespace cv;
using namespace std;

namespace gyrocam
{
	Point3f toProjective(Vec4i segment, bool first = true);
	Point3f toProjective(float x, float y);
	double norm12(Point3f p);
	bool isIncident(Point3f vp, LineSegment segment);
	void setRow(Mat a, int i, Mat col);
	
	int iterateOnSet(set<int> &base, set<int>::iterator &it, int shift);
}