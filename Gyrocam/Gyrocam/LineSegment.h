#pragma once
#include "opencv2/core/core.hpp"

namespace gyrocam
{
	class LineSegment
	{ 
	
	public:
		cv::Vec4i origin;
		cv::Point3d from, to, middle, line;

		LineSegment(const cv::Vec4i &lineSegment);
		bool isIncident(const cv::Point3d &vp, double ANGLE_EPSILON) const;
		~LineSegment(void);

		static std::vector<LineSegment> toLineSegments(const std::vector<cv::Vec4i> &lineSegments, double minLineSegmentLength);
	};

}