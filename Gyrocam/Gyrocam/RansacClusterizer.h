#pragma once
#include <set>
#include "opencv2/core/core.hpp"

#include "LineSegment.h"


namespace gyrocam
{
	namespace ransac
	{
		//int iterateOnSet(const std::set<int> &base, std::set<int>::iterator &it, int shift);
		int countInducedSegments(const std::vector<LineSegment> &segments, const std::set<int> &notUsed, const cv::Point3d &vp, double angleEpsilon);
		std::vector<int> getInducedSegments(const std::vector<LineSegment> &segments, const std::set<int> &notUsed, const cv::Point3d &vp, double angleEpsilon);
		std::vector<LineSegment> resolveIndices(const std::vector<LineSegment> &segments, const std::vector<int> &indices);
		void markInducedSegmentsAsUsed(std::set<int> &notUsed, const std::vector<int> &toErase);
	
		std::vector<LineSegment> nextCluster(const std::vector<LineSegment> &segments, const std::set<int> &notUsed, cv::Point3d &outVanishingPoint);
	}
}