#pragma once
#include <set>
#include "opencv2/core/core.hpp"

#include "LineSegment.h"

using namespace std;
using namespace cv;

namespace gyrocam
{
	static const double ransacNeededProbability = 0.95;

	class RansacClusterizer
	{
		bool isNotEnoughIterations(int c, int n, int iteration);
		Point3f nextPossibleVanishingPoint(int n);

	public:
		vector<LineSegment> segments;
		set<int> notUsed;

		RansacClusterizer(vector<LineSegment> segments);
		vector<LineSegment> nextCluster(Point3f &outVanishingPoint);
		~RansacClusterizer(void);
	};

}