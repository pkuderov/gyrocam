#include "Geometry.h"
#include "Config.h"

namespace gyrocam
{
	Point3d lineThroughPoints(Point3d p1, Point3d p2)
	{
		return p1.cross(p2);
	}

	Point3d intersection(Point3d l1, Point3d l2)
	{
		return l1.cross(l2);
	}

	double incidence(Point3d p1, Point3d p2)
	{
		return p1.ddot(p2);
	}

	bool isInfinitePoint(Point3d p)
	{
		return abs(p.z) < infinitePointEpsilon * norm(p);
	}

	
	void normalizeZ(Point3d &p)
	{
		if (!isInfinitePoint(p))
			p /= p.z;
	}
}