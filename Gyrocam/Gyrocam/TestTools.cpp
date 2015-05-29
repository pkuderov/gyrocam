#include "TestTools.h"
#include <iostream>

#include "opencv2/imgproc/imgproc.hpp"

namespace gyrocam
{
	void testOut(Vec4i s)
	{
		std::cout << "[(" << s[0] << ", " << s[1] << "), (" << s[2] << ", " << s[3] << ")]";
	}

	void testOut(Point3d p)
	{
		std::cout << p;
	}
	
	vector<Scalar> getColors()
	{
		vector<Scalar> colors;
		Scalar blue(255, 0, 0);
		Scalar green(0, 255, 0);
		Scalar red(0, 0, 255);
	
		colors.push_back(blue);
		colors.push_back(green);
		colors.push_back(red);
		colors.push_back(blue + green);
		colors.push_back(blue + red);
		colors.push_back(green + red);
		return colors;
	}

	void drawFoundSegments(vector<LineSegment> segments, Mat image, Scalar color, int thickness)
	{
		for (int i = 0; i < segments.size(); i++)
		{
			Vec4i origin = segments[i].origin;
			line(image, Point(origin[0], origin[1]), Point(origin[2], origin[3]), color, thickness);
		}
	}
	void drawPointMarkerIfVisibleOnImage(Point3d p, Mat image, Scalar color)
	{
		if (p.x < 0 || p.x > image.cols || p.y < 0 || p.y > image.rows)
			return;

		double sq2 = sqrt(2.0);
		double r = 10;
		double d = r*sq2/2;

		circle(image, Point2d(p.x, p.y), r, color, 2);
		line(image, Point2d(p.x-d, p.y+d), Point2d(p.x+d, p.y-d), color, 2);
		line(image, Point2d(p.x-d, p.y-d), Point2d(p.x+d, p.y+d), color, 2);
	}
}