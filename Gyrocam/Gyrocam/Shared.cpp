#include "Shared.h"
#include "Geometry.h"
#include "Config.h"

namespace gyrocam
{
	Point3d toProjective(Vec4i segment, bool first)
	{
		return first
			? toProjective(segment[0], segment[1])
			: toProjective(segment[2], segment[3]);
	}
	Point3d toProjective(double x, double y)
	{
		return Point3d(x, y, 1);
	}
	
	double norm12(Point3d p)
	{
		return sqrt(p.x*p.x + p.y*p.y);
	}
	
	bool isIncident(Point3d vp, LineSegment segment)
	{
		Point3d l = lineThroughPoints(segment.middle, vp);
		double d = incidence(l, segment.from) / norm12(l);
		return abs(d) <= distanceEpsilon 
			&& abs(asin(d / norm(segment.middle - segment.from))) <= angleEpsilon;
	}

	void setRow(Mat a, int i, Mat col)
	{
		for (int j = 0; j < col.rows; j++)
			a.at<double>(i, j) = col.at<double>(j, 0);
	}

	
	int iterateOnSet(set<int> &base, set<int>::iterator &it, int shift)
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
	
	
	Mat toNormalized(Mat line, Mat invCalibrationMatrix)
	{
		return invCalibrationMatrix * line;
	}

	Mat toNormalized(LineSegment s, Mat invCalibrationMatrix)
	{
		Mat from(s.from);
		Mat to(s.to);
		from = toNormalized(from, invCalibrationMatrix);
		to = toNormalized(to, invCalibrationMatrix);

		Mat line = to.cross(from);
		line /= norm(line);
		return line;
	}

	Point3d fromNormalized(Mat line, Mat calibrationMatrix)
	{
		Mat t = calibrationMatrix * line;
		return Point3d(t);
	}

	Mat getNearestOrthogonalMatrix(Mat a)
	{
		/*Mat t = a.col(0).cross(a.col(1)).t() * a.col(2);
		double d = t.at<double>(0, 0);
		if (abs(d) < 0.4 && d < 0);
		if (abs(d) > 0.4)
			return 1

		for (int i = 0; i < 3; i++)
			for (int j = i + 1; j< 3; j++)
				if (acos(norm(a.col(j).t() * a.col(i))) > 0.4)
*/
		SVD svd(a, SVD::FULL_UV);
		return svd.u * svd.vt;
	}

	// ----------------- OBSOLETE --------------------------

	// Finds the intersection of two lines, or returns false.
	// The lines are defined by (o1, p1) and (o2, p2).
	bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2, Point2f &r)
	{
		Point2f x = o2 - o1;
		Point2f d1 = p1 - o1;
		Point2f d2 = p2 - o2;

		float cross = d1.x*d2.y - d1.y*d2.x;
		if (abs(cross) < /*EPS*/1e-8)
			return false;

		double t1 = (x.x * d2.y - x.y * d2.x)/cross;
		r = o1 + d1 * t1;
		return true;
	}

	double angleBetween(Point2f v1, Point2f v2)
	{
		return v1.ddot(v2) / (norm(v1) * norm(v2));
	}
}