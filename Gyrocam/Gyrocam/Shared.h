#pragma once
#include<set>

#include "opencv2/core/core.hpp"
#include "LineSegment.h"

namespace gyrocam
{
	cv::Point3d toProjective(const cv::Vec4i &segment, bool first = true);

	void setRow(cv::Mat &a, int i, const cv::Mat &col);
	double angleBetween(const cv::Mat &r, const cv::Mat &c);
	double angleBetweenAbs(const cv::Mat &r, const cv::Mat &c);
	cv::Mat getNearestOrthogonalMatrix(const cv::Mat &a);
	void findMaxOnRow(const cv::Mat &a, double &m, int &i);
	void swapColumns(cv::Mat &a, int i, int j);
	void reorderColumn(cv::Mat &a, int row);
	void reorderColumn(cv::Mat &a);

	cv::Mat getEulerAngles(const cv::Mat &r);

	cv::Mat resizeImage(const cv::Mat &image, bool isPocketSize);
	double findMinAllowedLineSegmentLength(const cv::Mat &image);
}