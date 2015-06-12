#include "Shared.h"
#include "Geometry.h"
#include "Config.h"

#include "opencv2/imgproc/imgproc.hpp"

namespace gyrocam
{
	cv::Point3d toProjective(const cv::Vec4i &segment, bool first)
	{
		return first
			? toProjective(segment[0], segment[1])
			: toProjective(segment[2], segment[3]);
	}

	void setRow(cv::Mat &a, int i, const cv::Mat &col)
	{
		a.row(i) = col.t();
	}
	
	double angleBetween(const cv::Mat &r, const cv::Mat &c)
	{
		cv::Mat x = r * c;
		double d = x.at<double>(0, 0);
		return acos(d);
	}
	double angleBetweenAbs(const cv::Mat &r, const cv::Mat &c)
	{
		return abs(CV_PI/2 - abs(angleBetween(r, c) - CV_PI/2));
	}

	cv::Mat getNearestOrthogonalMatrix(const cv::Mat &a)
	{
		cv::SVD svd(a, cv::SVD::FULL_UV);
		return svd.u * svd.vt;
	}
	
	void findMaxOnRow(const cv::Mat &a, double &m, int &i)
	{
		m = 0;	i = 0;
		for (int c = 0; c < 3; c++)
		{
			double x = abs(a.at<double>(0, c));
			if (x > m)
			{
				m = x; 
				i = c;
			}
		}
	}

	void swapColumns(cv::Mat &a, int i, int j)
	{
		if (i == j)
			return;
	
		cv::Mat t1, t2; 
		a.col(i).copyTo(t1);
		a.col(j).copyTo(t2);

		t1.copyTo(a.col(j));
		t2.copyTo(a.col(i));
	}

	void reorderColumn(cv::Mat &a, int row)
	{
		double m;
		int i;
		findMaxOnRow(a.row(row), m, i);
		swapColumns(a, row, i);
		if (a.at<double>(row, row) < 0)
			a.col(row) = -1 * a.col(row);
	}
	
	void reorderColumn(cv::Mat &a)
	{
		cv::Mat t = a.col(0).cross(a.col(1)).t() * a.col(2);
		if (t.at<double>(0, 0) < 0)
			a.col(2) = -1 * a.col(2);
	}

	cv::Mat getEulerAngles(const cv::Mat &r)
	{
		cv::Mat a = cv::Mat::eye(1, 3, CV_64FC1);
		a.at<double>(0, 0) = atan2(r.at<double>(1,2), r.at<double>(2, 2));
		a.at<double>(0, 1) = -asin(r.at<double>(0,2));
		a.at<double>(0, 2) = atan2(r.at<double>(0,1), r.at<double>(0, 0));

		a = a * 180 / CV_PI;
		return a;
	}

	cv::Mat resizeImage(const cv::Mat &image, bool isPocketSize)
	{	
		cv::Mat resultImage;
		double width = POCKET_IMAGE_WIDTH * (isPocketSize ? 1 : BIG_IMAGE_SCALE);
		double height = POCKET_IMAGE_HEIGHT * (isPocketSize ? 1 : BIG_IMAGE_SCALE);
		double scaleFactor = std::min(width/image.cols, height / image.rows);
		if (scaleFactor < 1.0)
			resize(image, resultImage, cv::Size(), scaleFactor, scaleFactor);
		else
			resultImage = image;
		return resultImage;
	}

	double findMinAllowedLineSegmentLength(const cv::Mat &image)
	{
		double wScale = image.cols / POCKET_IMAGE_WIDTH;
		double hScale = image.rows / POCKET_IMAGE_HEIGHT;
		return std::max(wScale, hScale) * MIN_ALLOWED_LINE_SEGMENT_LENGTH;
	}
}