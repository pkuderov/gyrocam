#include "RansacClusterizer.h"
#include "Shared.h"
#include "Geometry.h"
#include "Config.h"

namespace gyrocam
{	
	cv::Point3d nextPossibleVanishingPoint(const std::vector<LineSegment> &segments, const std::set<int> &notUsed, int n);
	bool isNotEnoughIterations(int c, int n, int iteration);
	int iterateOnSet(const std::set<int> &base, std::set<int>::iterator &it, int shift);


	int ransac::countInducedSegments(const std::vector<LineSegment> &segments, const std::set<int> &notUsed, const cv::Point3d &vp, double angleEpsilon)
	{
		int count = 0;
		for (std::set<int>::iterator it = notUsed.begin(); it != notUsed.end(); it++)
		{
			if (segments[*it].isIncident(vp, angleEpsilon))
				count++;
		}
		return count;
	}
	std::vector<int> ransac::getInducedSegments(const std::vector<LineSegment> &segments, const std::set<int> &notUsed, const cv::Point3d &vp, double angleEpsilon)
	{
		std::vector<int> res;
		for (std::set<int>::iterator it = notUsed.begin(); it != notUsed.end(); ++it)
		{
			if (segments[*it].isIncident(vp, angleEpsilon))
				res.push_back(*it);
		}
		return res;
	}
	
	std::vector<LineSegment> ransac::resolveIndices(const std::vector<LineSegment> &segments, const std::vector<int> &indices)
	{
		std::vector<LineSegment> res;
		for (int i = 0; i < indices.size(); i++)
			res.push_back(segments[indices[i]]);

		return res;
	}
	void ransac::markInducedSegmentsAsUsed(std::set<int> &notUsed, const std::vector<int> &toErase)
	{
		for (int i = 0; i < toErase.size(); i++)
			notUsed.erase(toErase[i]);
	}
	
	std::vector<LineSegment> ransac::nextCluster(const std::vector<LineSegment> &segments, const std::set<int> &notUsed, cv::Point3d &outVanishingPoint)
	{
		int n = notUsed.size();
		int bestCount = 0;
		int iteration = 0;

		do
		{
			cv::Point3d vp = nextPossibleVanishingPoint(segments, notUsed, n);
			normalizeZ(vp);

			int count = countInducedSegments(segments, notUsed, vp, ANGLE_EPSILON);
			if (count > bestCount)
			{
				bestCount = count;
				outVanishingPoint = vp;
			}

			iteration++;
		}
		while (isNotEnoughIterations(bestCount, n, iteration));
	
		std::vector<LineSegment> found;
		if (bestCount == 0)
			return found;

		return resolveIndices(segments, getInducedSegments(segments, notUsed, outVanishingPoint, ANGLE_EPSILON));
	}

	cv::Point3d nextPossibleVanishingPoint(const std::vector<LineSegment> &segments, const std::set<int> &notUsed, int n)
	{
		std::set<int>::iterator it = notUsed.begin();

		int i = rand() % n;
		int j = (i + 1 + rand() % (n - 1)) % n;

		int iInd = iterateOnSet(notUsed, it, i);
		int jInd = iterateOnSet(notUsed, it, j);

		return intersection(segments[iInd].line, segments[jInd].line);
	}

	bool isNotEnoughIterations(int c, int n, int iteration)
	{
		if (iteration == 0)
			return true;
		if (c == 0 || n == 0)
			return false;

		double r = (double)c / n;
		double logProb = log(1 - RANSAC_PROBABILITY);
		double logChoose = log(1 - r * r);
		return iteration < logProb / logChoose;
	}

	int iterateOnSet(const std::set<int> &base, std::set<int>::iterator &it, int shift)
	{
		while (shift > 0)
		{
			if (it == base.end())
				it = base.begin();

			shift --;
			it++;
		}

		if (it == base.end())
			it = base.begin();	

		return *it;
	}
}