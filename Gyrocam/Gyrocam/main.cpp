#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <set>

#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "Config.h"
#include "RansacClusterizer.h"

#include "TestTools.h"
#include "Geometry.h"
#include "Shared.h"

using namespace std;
using namespace cv;
using namespace gyrocam;

vector<Vec4i> getSegments(Mat image)
{
    vector<Vec4i> lines_std;

    // Detect the lines
    Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
    ls->detect(image, lines_std);

	return lines_std;
}

vector<LineSegment> toProjectives(vector<Vec4i> lineSegments)
{
	vector<LineSegment> _lineSegments;
	for (int i = 0; i < lineSegments.size(); i++)
	{
		LineSegment segment(lineSegments[i]);
		if (norm(segment.to - segment.from) >= minAllowedLineSegmentLength)
			_lineSegments.push_back(segment);
	}
	return _lineSegments;
}


Mat getRotationMatrixBasedOnVanishingPoints(vector<Point3d> vps)
{
	Mat A = Mat::eye(3, 3, CV_64FC1);
	for (int i = 0; i < vps.size(); i++)
		setRow(A, i, Mat(vps[i] / norm(vps[i])));

	return A;
}

Mat orthogonalizeBasis(Mat a)
{
	SVD svd(a, SVD::FULL_UV);
	return svd.u * svd.vt;
}

RansacClusterizer initRansacClusterizer(Mat image)
{
	srand (time(NULL));
	vector<Vec4i> rawlineSegments = getSegments(image);
	
	vector<LineSegment> lineSegments = toProjectives(rawlineSegments);
	RansacClusterizer clusterizer(lineSegments);
	return clusterizer;
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

void drawFoundSegments(vector<LineSegment> segments, Mat image, Scalar color, int thickness = 2)
{
	for (int i = 0; i < segments.size(); i++)
	{
		Vec4i origin = segments[i].origin;
		line(image, Point(origin[0], origin[1]), Point(origin[2], origin[3]), color, thickness);
	}
}

Mat readCalibrationMatrix(std::string calibrationMatrixPath)
{
	std::string temp;
	std::ifstream infile(calibrationMatrixPath);

	double focalLength, pixelSize, ppHor, ppVer;

	infile >> temp >> focalLength;
	infile >> temp >> pixelSize;
	infile >> temp >> ppHor >> ppVer;
	infile.close();

	Mat calibrationMatrix = Mat::eye(3, 3, CV_64FC1);
	calibrationMatrix.at<double>(0, 0) = focalLength / pixelSize;
	calibrationMatrix.at<double>(1, 1) = focalLength / pixelSize;
	calibrationMatrix.at<double>(0, 2) = ppHor - 320;
	calibrationMatrix.at<double>(1, 2) = ppVer - 240;

	return calibrationMatrix;
}

Mat toNormalized(Point3d l, Mat invCalibrationMatrix)
{
	Point3d t(l);
	normalizeZ(t);

	Mat line(t);
	line = invCalibrationMatrix * line;
	line /= norm(line);
	return line;
}

Mat toNormalized(Mat line, Mat invCalibrationMatrix)
{
	return invCalibrationMatrix * line;
}

Point3d fromNormalized(Mat line, Mat calibrationMatrix)
{
	Mat t = calibrationMatrix * line;
	return Point3d(t);
}

void solveZManually(Mat A, Mat &res)
{
	SVD svd(A, SVD::FULL_UV);
	cout << svd.w << endl;
	cout << svd.vt << endl;
	double normW = norm(svd.w);
	for (int i = 0; i < svd.w.rows; i++)
	{
		double d = svd.w.at<double>(i, 0);
		if (d < 0)
			cout << "false";
		if (d < normW * 1e-4)
		{
			res = svd.vt.row(i == 0 ? i : i - 1);
			return;
		}
	}

	res = svd.vt.row(svd.vt.cols - 1);
}

Point3d refineVanishingPoint(vector<LineSegment> segments, Mat inversedCalibrationMatrix)
{
	Mat A = Mat::zeros(segments.size(), 3, CV_64FC1);
	for (int i = 0; i < segments.size(); i++)
	{
		setRow(A, i, toNormalized(segments[i].line, inversedCalibrationMatrix));
	}

	Mat res = Mat::zeros(3, 1, CV_64FC1);
	solveZManually(A, res);
	std::cout << res << endl;

	Mat res1 = Mat::zeros(3, 1, CV_64FC1);
	SVD::solveZ(A, res1);
	std::cout << res1 << endl;
	cout << endl;
	return Point3d(res);
}

Point3d refineVanishingPoint1(vector<LineSegment> segments, Mat cM, Mat inversedCalibrationMatrix)
{
	Mat cmr = Mat::zeros(3, 4, CV_64FC1);
	cM.copyTo(cmr(Rect(0, 0, 3, 3)));
	std::cout << cmr << endl;
	Mat icmr = Mat::zeros(4, 3, CV_64FC1);
	inversedCalibrationMatrix.copyTo(icmr(Rect(0, 0, 3, 3)));
	std::cout << icmr << endl;

	Mat icmr2 = cmr.inv(DECOMP_SVD);
	std::cout << icmr2 << endl;

	Mat A = Mat::zeros(segments.size(), 4, CV_64FC1);
	for (int i = 0; i < segments.size(); i++)
	{
		Point3d t(segments[i].line);
		normalizeZ(t);

		Mat line(t);
		line = icmr * line;
		line /= norm(line);

		Mat l = line(Rect(0, 0, 1, 3));
		Point3d p(l);
		Point3d q(toNormalized(segments[i].line, inversedCalibrationMatrix));

		setRow(A, i, line);
	}

	Mat res = Mat::zeros(A.cols, 1, CV_64FC1);
	SVD::solveZ(A, res);
	std::cout << res << endl;
	if (abs(res.at<double>(3, 0)) > 1e-12 * norm(res))
		res /= res.at<double>(3, 0);
	Mat vres = res(Rect(0, 0, 1, 3));
	return Point3d(vres);
}

vector<LineSegment> getSegmentsIncidentWithVanishingPoint(Point3d vp, vector<LineSegment> segments)
{
	vector<LineSegment> res;
	for (int i = 0; i < segments.size(); i++)
	{
		if (isIncident(vp, segments[i]))
			res.push_back(segments[i]);
	}

	return res;
}

Point3d refineVanishingPointWithoutUncalibration(vector<LineSegment> segments)
{
	Mat A = Mat::zeros(segments.size(), 3, CV_64FC1);
	for (int i = 0; i < segments.size(); i++)
	{
		Point3d originLine(segments[i].line);
		originLine /= norm(originLine);
		setRow(A, i, Mat(originLine));
	}

	Mat res;
	SVD::solveZ(A, res);
	return Point3d(res);
}

void processImageWithouthCalibration(std::string in, std::string out)
{
    Mat image = imread(in, 0);
	RansacClusterizer clusterizer = initRansacClusterizer(image);

	image = imread(in, 1);
	vector<Scalar> colors = getColors();
	Scalar black(0, 0, 0);
	drawFoundSegments(clusterizer.segments, image, black, 1);

	vector<Point3d> vps;
	for (int i = 0; i < 3; i++)
	{
		if (clusterizer.notUsed.size() == 0)
			break;

		Point3d originVp;
		vector<LineSegment> found = clusterizer.nextCluster(originVp);
		drawFoundSegments(found, image, colors[i] / 2);

		Point3d refinedVp = refineVanishingPointWithoutUncalibration(found);
		drawFoundSegments(getSegmentsIncidentWithVanishingPoint(refinedVp, clusterizer.segments), image, colors[i]);
		vps.push_back(refinedVp);
	}

	imwrite(out, image);

	Mat vpBasis = getRotationMatrixBasedOnVanishingPoints(vps);
	Mat orthoVpBasis = orthogonalizeBasis(vpBasis);
	
	std::ofstream outfile(out + ".txt");
	outfile << vpBasis << endl << orthoVpBasis << endl;
	outfile.flush();
	outfile.close();

	std::cout << vpBasis << endl;
	std::cout << orthoVpBasis << endl;

	imshow("result", image);
}

void processImage(std::string in, std::string out, Mat calibrationMatrix, Mat inversedCalibrationMatrix)
{
    Mat image = imread(in, 0);
	RansacClusterizer clusterizer = initRansacClusterizer(image);

	image = imread(in, 1);
	vector<Scalar> colors = getColors();

	vector<Point3d> vps;
	for (int i = 0; i < 3; i++)
	{
		if (clusterizer.notUsed.size() == 0)
			break;

		Point3d originVp;
		vector<LineSegment> found = clusterizer.nextCluster(originVp);
		drawFoundSegments(found, image, colors[i] / 2);

		Point3d refinedVp = refineVanishingPoint(found, inversedCalibrationMatrix);
		Point3d p = fromNormalized(Mat(refinedVp), calibrationMatrix);
		normalizeZ(p);
		drawFoundSegments(getSegmentsIncidentWithVanishingPoint(p, clusterizer.segments), image, colors[i]);
		vps.push_back(refinedVp);
	}

	imwrite(out, image);

	Mat vpBasis = getRotationMatrixBasedOnVanishingPoints(vps);
	Mat orthoVpBasis = orthogonalizeBasis(vpBasis);

	std::ofstream outfile(out + ".txt");
	outfile << vpBasis << endl << orthoVpBasis << endl;
	outfile.flush();
	outfile.close();

	std::cout << vpBasis << endl;
	std::cout << orthoVpBasis << endl;

	imshow("result", image);
}


int main(int argc, char** argv)
{
    std::string in = "../../TestSamples/urban3.jpg";
	std::string out = "../../TestSamples/output.jpg";
	std::string calibrationMatrixPath = "../../TestSamples/YorkUrbanDB/cameraParameters.txt";
    if (argc >= 2)
    {
        in = argv[1];
    }
	if (argc >= 3)
	{
		out = argv[2];
	}
	if (argc >= 4)
	{
		calibrationMatrixPath = argv[3];
	}

	
	Mat calibrationMatrix = readCalibrationMatrix(calibrationMatrixPath);
	Mat inversedCalibrationMatrix = calibrationMatrix.inv();

	processImageWithouthCalibration(in, out);	
	//processImage(in, out, calibrationMatrix, inversedCalibrationMatrix);

	waitKey();

    return 0;
}