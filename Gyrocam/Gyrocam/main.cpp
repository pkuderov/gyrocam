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


Mat getRotationMatrixBasedOnVanishingPoints(vector<Point3f> vps)
{
	Mat A = Mat::eye(3, 3, CV_32FC1);
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

	Mat calibrationMatrix = Mat::eye(3, 3, CV_32FC1);
	calibrationMatrix.at<float>(0, 0) = focalLength / pixelSize;
	calibrationMatrix.at<float>(1, 1) = focalLength / pixelSize;
	calibrationMatrix.at<float>(0, 2) = ppHor - 320;
	calibrationMatrix.at<float>(1, 2) = ppVer - 240;

	return calibrationMatrix;
}

Mat toNormalized(Point3f l, Mat invCalibrationMatrix)
{
	Point3f t(l);
	normalizeZ(t);

	Mat line(t);
	line = invCalibrationMatrix * line;
	line /= norm(Point3f(line));
	return line;
}

Mat toNormalized(Mat line, Mat invCalibrationMatrix)
{
	return invCalibrationMatrix * line;
}

Point3f fromNormalized(Mat line, Mat calibrationMatrix)
{
	Mat t = calibrationMatrix * line;
	return Point3f(t);
}


Point3f refineVanishingPoint(vector<LineSegment> segments, Mat inversedCalibrationMatrix)
{
	Mat A = Mat::zeros(segments.size(), 3, CV_32FC1);
	for (int i = 0; i < segments.size(); i++)
	{
		setRow(A, i, toNormalized(segments[i].line, inversedCalibrationMatrix));
	}

	Mat res;
	SVD::solveZ(A, res);
	return Point3f(res);
}

vector<LineSegment> getSegmentsIncidentWithVanishingPoint(Point3f vp, vector<LineSegment> segments)
{
	vector<LineSegment> res;
	for (int i = 0; i < segments.size(); i++)
	{
		if (isIncident(vp, segments[i]))
			res.push_back(segments[i]);
	}

	return res;
}

Point3f refineVanishingPointWithoutUncalibration(vector<LineSegment> segments)
{
	Mat A = Mat::zeros(segments.size(), 3, CV_32FC1);
	for (int i = 0; i < segments.size(); i++)
	{
		Point3f originLine(segments[i].line);
		originLine /= norm(originLine);
		setRow(A, i, Mat(originLine));
	}

	Mat res;
	SVD::solveZ(A, res);
	return Point3f(res);
}

void processImageWithouthCalibration(std::string in, std::string out)
{
    Mat image = imread(in, 0);
	RansacClusterizer clusterizer = initRansacClusterizer(image);

	image = imread(in, 1);
	vector<Scalar> colors = getColors();
	Scalar black(0, 0, 0);
	//drawFoundSegments(clusterizer.segments, image, black, 1);

	vector<Point3f> vps;
	for (int i = 0; i < 3; i++)
	{
		if (clusterizer.notUsed.size() == 0)
			break;

		Point3f originVp;
		vector<LineSegment> found = clusterizer.nextCluster(originVp);
		drawFoundSegments(found, image, colors[i] / 2);

		Point3f refinedVp = refineVanishingPointWithoutUncalibration(found);
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

	vector<Point3f> vps;
	for (int i = 0; i < 3; i++)
	{
		if (clusterizer.notUsed.size() == 0)
			break;

		Point3f originVp;
		vector<LineSegment> found = clusterizer.nextCluster(originVp);
		drawFoundSegments(found, image, colors[i] / 2);

		Point3f refinedVp = refineVanishingPoint(found, inversedCalibrationMatrix);
		Point3f p = fromNormalized(Mat(refinedVp), calibrationMatrix);
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