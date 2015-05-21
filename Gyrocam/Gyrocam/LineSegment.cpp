#include "LineSegment.h"
#include "Shared.h"
#include "Geometry.h"

namespace gyrocam
{
	LineSegment::LineSegment(Vec4i lineSegment)
	{
		origin = lineSegment;

		from = toProjective(origin);
		to = toProjective(origin, false);
		line = lineThroughPoints(from, to);
		middle = (from + to) / 2;
	}

	LineSegment::~LineSegment(void)
	{
	}
}