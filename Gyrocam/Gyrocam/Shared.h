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

	int iterateOnSet(set<int> &base, set<int>::iterator &it, int shift);
	
	void setRow(Mat a, int i, Mat col);
	Mat toNormalized(Mat line, Mat invCalibrationMatrix);
	Mat toNormalized(LineSegment s, Mat invCalibrationMatrix);
	Point3d fromNormalized(Mat line, Mat calibrationMatrix);
	Mat getNearestOrthogonalMatrix(Mat a);
}