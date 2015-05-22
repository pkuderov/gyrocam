#pragma once
#include "opencv2/core/core.hpp"

using namespace cv;
using namespace std;

namespace gyrocam
{
	class LineSegment
	{ 
	
	public:
		Vec4i origin;
		Point3d from, to, middle, line;

		LineSegment(Vec4i lineSegment);
		~LineSegment(void);
	};

}