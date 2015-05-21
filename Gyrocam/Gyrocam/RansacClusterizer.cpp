#include "RansacClusterizer.h"
#include "Shared.h"
#include "Geometry.h"
#include "Config.h"

namespace gyrocam
{
	RansacClusterizer::RansacClusterizer(vector<LineSegment> segments)
	{
		RansacClusterizer::segments = segments;
		
		notUsed = set<int>();
		for (int i = 0; i < segments.size(); i++)
			notUsed.insert(i);
	}

	vector<LineSegment> RansacClusterizer::nextCluster(Point3f &outVanishingPoint)
	{
		int n = notUsed.size();
		int bestCount = 0;
		Point3f bestVanishingPoint;
		int iteration = 0;

		while(isNotEnoughIterations(bestCount, n, iteration))
		{
			Point3f vp = nextPossibleVanishingPoint(n);
			normalizeZ(vp);

			int count = 0;
			for (set<int>::iterator it = notUsed.begin(); it != notUsed.end(); it++)
			{
				if (isIncident(vp, segments[*it]))
					count++;
			}
			if (count > bestCount)
			{
				bestCount = count;
				bestVanishingPoint = vp;
			}

			iteration++;
		}
	
		vector<LineSegment> found;
		if (bestCount == 0)
			return found;
	
		for (set<int>::iterator it = notUsed.begin(); it != notUsed.end();)
		{
			if (isIncident(bestVanishingPoint, segments[*it]))
			{
				found.push_back(segments[*it]);
				set<int>::iterator tmp = it;
				++tmp;
				notUsed.erase(it);
				it = tmp;
			}
			else
				++it;
		}

		outVanishingPoint = bestVanishingPoint;
		return found;
	}

	Point3f RansacClusterizer::nextPossibleVanishingPoint(int n)
	{
		set<int>::iterator it = notUsed.begin();

		int i = rand() % n;
		int j = (i + 1 + rand() % (n - 1)) % n;

		int iInd = iterateOnSet(notUsed, it, i);
		int jInd = iterateOnSet(notUsed, it, j);

		return intersection(segments[iInd].line, segments[jInd].line);
	}

	bool RansacClusterizer::isNotEnoughIterations(int c, int n, int iteration)
	{
		if (c == 0)
			return true;

		double r = (double)c / n;
		double logProb = log(1 - ransacNeededProbability);
		double logChoose = log(1 - r * r);
		return iteration < logProb / logChoose;
	}

	RansacClusterizer::~RansacClusterizer(void)
	{
	}
}