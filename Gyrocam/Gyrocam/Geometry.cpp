#include "Geometry.h"
#include "Config.h"

namespace gyrocam
{
	cv::Point3d toProjective(double x, double y)
	{
		return cv::Point3d(x, y, 1);
	}
	
	double norm12(const cv::Point3d &p)
	{
		return sqrt(p.x*p.x + p.y*p.y);
	}
	
	cv::Point3d lineThroughPoints(const cv::Point3d &p1, const cv::Point3d &p2)
	{
		return p1.cross(p2);
	}

	cv::Point3d intersection(const cv::Point3d &l1, const cv::Point3d &l2)
	{
		return l1.cross(l2);
	}

	double incidence(const cv::Point3d &p1, const cv::Point3d &p2)
	{
		return p1.ddot(p2);
	}

	bool isInfinitePoint(const cv::Point3d &p)
	{
		return abs(p.z) < INFINITE_POINT_Z_EPSILON * norm12(p);
	}

	void normalizeZ(cv::Point3d &p)
	{
		if (!isInfinitePoint(p))
			p /= p.z;
	}
}