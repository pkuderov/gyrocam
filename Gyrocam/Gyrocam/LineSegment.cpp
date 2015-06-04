#include "LineSegment.h"
#include "Shared.h"
#include "Geometry.h"

namespace gyrocam
{
	LineSegment::LineSegment(const cv::Vec4i &lineSegment)
	{
		origin = lineSegment;

		from = toProjective(origin);
		to = toProjective(origin, false);
		line = lineThroughPoints(from, to);
		middle = (from + to) / 2;
	}

	bool LineSegment::isIncident(const cv::Point3d &vp, double ANGLE_EPSILON) const
	{
		cv::Point3d l = lineThroughPoints(middle, vp);
		double d = incidence(l, from) / (norm12(l) * norm(middle - from));
		return abs(asin(d)) <= ANGLE_EPSILON;
	}


	LineSegment::~LineSegment(void)
	{
	}
	
	std::vector<LineSegment> LineSegment::toLineSegments(const std::vector<cv::Vec4i> &lineSegments, double minLineSegmentLength)
	{
		std::vector<LineSegment> _lineSegments;
		for (int i = 0; i < lineSegments.size(); i++)
		{
			LineSegment segment(lineSegments[i]);
			if (norm(segment.to - segment.from) >= minLineSegmentLength)
				_lineSegments.push_back(segment);
		}
		return _lineSegments;
	}
}