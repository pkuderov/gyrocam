#include "Geometry.h"
#include "Config.h"

namespace gyrocam
{
	Point3f lineThroughPoints(Point3f p1, Point3f p2)
	{
		return p1.cross(p2);
	}

	Point3f intersection(Point3f l1, Point3f l2)
	{
		return l1.cross(l2);
	}

	double incidence(Point3f p1, Point3f p2)
	{
		return p1.ddot(p2);
	}

	bool isInfinitePoint(Point3f p)
	{
		return abs(p.z) < infinitePointEpsilon * norm(p);
	}

	
	void normalizeZ(Point3f &p)
	{
		if (!isInfinitePoint(p))
			p /= p.z;
	}
}