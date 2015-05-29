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

bool TRACE_ENABLED = false;
bool DRAW_UNREFINED = false;
bool DRAW_REFINED = true;
bool SILENT_MODE = false;
bool YORK_URBAN_DB_TEST_MODE = false;

bool IS_TRACE_ENABLED() { return TRACE_ENABLED && !SILENT_MODE; }
bool SHOULD_DRAW_UNREFINED() { return DRAW_UNREFINED; }
bool SHOULD_DRAW_REFINED() { return DRAW_REFINED; }

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

	return A.t();
}

RansacClusterizer initRansacClusterizer(Mat image)
{
	srand (time(NULL));
	vector<Vec4i> rawlineSegments = getSegments(image);
	
	vector<LineSegment> lineSegments = toProjectives(rawlineSegments);
	RansacClusterizer clusterizer(lineSegments);
	return clusterizer;
}

Mat readCalibrationMatrix(bool calibMatrixIsSet, std::string calibrationMatrixPath)
{	
	Mat calibrationMatrix = Mat::eye(3, 3, CV_64FC1);
	if (!calibMatrixIsSet)
		return calibrationMatrix;

	std::string temp;
	std::ifstream infile(calibrationMatrixPath);

	double focalLength, pixelSize, ppHor, ppVer;

	infile >> temp >> focalLength;
	infile >> temp >> pixelSize;
	infile >> temp >> ppHor >> ppVer;
	infile.close();

	calibrationMatrix.at<double>(0, 0) = focalLength / pixelSize;
	calibrationMatrix.at<double>(1, 1) = focalLength / pixelSize;
	calibrationMatrix.at<double>(0, 2) = ppHor;
	calibrationMatrix.at<double>(1, 2) = ppVer;
	return calibrationMatrix;
}

void solveZManually(Mat A, Mat &res)
{
	SVD svd(A, SVD::FULL_UV);

	if (IS_TRACE_ENABLED())
	{
		cout << "SVD results:" << endl;
		cout << "W: " << svd.w << endl;
		cout << "Vt: " << svd.vt << endl;
	}

	double maxSigma = svd.w.rows > 0 ? svd.w.at<double>(0, 0) : 1;
	for (int i = 0; i < svd.w.rows; i++)
	{
		double d = svd.w.at<double>(i, 0);
		if (d / maxSigma < infinitePointEpsilon)
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
		setRow(A, i, toNormalized(segments[i], inversedCalibrationMatrix));
	}

	Mat res = Mat::zeros(3, 1, CV_64FC1);
	solveZManually(A, res);
	res = res.t();

	if (IS_TRACE_ENABLED())
	{
		Mat res1 = Mat::zeros(3, 1, CV_64FC1);
		SVD::solveZ(A, res1);

		std::cout << "SolveZManually: " << res << endl;
		std::cout << "SolveZ: " << res1 << endl;
		cout << "SolveZ diff: " << norm(res - res1) << endl;
	}

	return Point3d(res);
}

vector<int> getSegmentsIncidentWithVanishingPoint(Point3d vp, vector<LineSegment> segments)
{
	vector<int> res;
	for (int i = 0; i < segments.size(); i++)
	{
		if (isIncident(vp, segments[i]))
			res.push_back(i);
	}

	return res;
}

vector<LineSegment> resolveIndices(vector<LineSegment> base, vector<int> indices)
{
	vector<LineSegment> res;
	for (int i = 0; i < indices.size(); i++)
		res.push_back(base[indices[i]]);

	return res;
}

void eraseFromSet(set<int> &s, vector<int> toErase)
{
	for (int i = 0; i < toErase.size(); i++)
		s.erase(toErase[i]);
}
	
void saveVanishingPointsDirections(string out, Mat vpBasis, Mat orthoVpBasis)
{
	int extInd = out.find_last_of('.');
	string outBase = (extInd == string::npos || (extInd + 1 < out.length() && out[extInd + 1] == '/')) 
		? out 
		: out.substr(0, extInd);
	
	// non-orthogonal basis
	std::ofstream outfile(outBase + "_gyrocam_vp_basis.txt");
	outfile << vpBasis << endl;
	outfile.flush();
	outfile.close();

	// orthogonal basis
	outfile = ofstream(outBase + "_gyrocam_vp_ortho_basis.txt");
	outfile << orthoVpBasis << endl;
	outfile.flush();
	outfile.close();

	if (TRACE_ENABLED)
	{
		std::cout << vpBasis << endl;
		std::cout << orthoVpBasis << endl;
	}
}

void processImage(std::string in, std::string out, Mat calibrationMatrix, Mat inversedCalibrationMatrix, Mat &vpBasis, Mat &vpOrthoBasis)
{
    Mat image = imread(in, 0);
	RansacClusterizer clusterizer = initRansacClusterizer(image);

	image = imread(in, 1);
	vector<Scalar> colors = getColors();
	
	if (SHOULD_DRAW_UNREFINED())
	{
		Scalar black(0, 0, 0);
		drawFoundSegments(clusterizer.segments, image, black, 1);
	}

	vector<Point3d> vps;
	for (int i = 0; i < 3; i++)
	{
		if (clusterizer.notUsed.size() < 2)
			break;

		Point3d originVp;
		vector<LineSegment> originCluster = clusterizer.nextCluster(originVp);
		if (originCluster.size() < 3)
			break;

		if (SHOULD_DRAW_UNREFINED())
		{
			drawFoundSegments(originCluster, image, colors[i] / 2);
			drawPointMarkerIfVisibleOnImage(originVp, image, colors[i] / 2);
		}

		Point3d refinedVpNormalized = refineVanishingPoint(originCluster, inversedCalibrationMatrix);
		Point3d refinedVp = fromNormalized(Mat(refinedVpNormalized), calibrationMatrix);
		normalizeZ(refinedVp);
		
		if (IS_TRACE_ENABLED())
		{
			cout << "OriginVP" << i << endl << originVp << endl;
			cout << "RefinedVP" << i << endl << refinedVp << endl;
		}

		vector<int> refinedClusterIndices = getSegmentsIncidentWithVanishingPoint(refinedVp, clusterizer.segments);
		eraseFromSet(clusterizer.notUsed, refinedClusterIndices);
		
		vector<LineSegment> refinedCluster = resolveIndices(clusterizer.segments, refinedClusterIndices);

		if (SHOULD_DRAW_REFINED())
		{
			drawPointMarkerIfVisibleOnImage(refinedVp, image, colors[i]);
			drawFoundSegments(refinedCluster, image, colors[i]);
		}

		vps.push_back(refinedVpNormalized);
	}

	if (SHOULD_DRAW_REFINED() || SHOULD_DRAW_UNREFINED())
	{
		imwrite(out, image);
		
		Mat showImage;
		double scaleFactor = min(1000.0/image.cols, 700.0 / image.rows);
		if (scaleFactor < 1.0)
			resize(image, showImage, Size(), scaleFactor, scaleFactor);
		else
			showImage = image;

		if (!SILENT_MODE)
			imshow("result", showImage);
	}
	
	vpBasis = getRotationMatrixBasedOnVanishingPoints(vps);
	vpOrthoBasis = getNearestOrthogonalMatrix(vpBasis);
}

Mat readVpBasis(string in)
{
	string temp;
	ifstream f(in);
	getline(f, temp);
	double t;
	Mat res = Mat::zeros(3, 3, CV_64FC1);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			f >> t;
			res.at<double>(i, j) = t;
		}

	return res;
}

Mat reorderColumnsInVpBasis(Mat b, Mat gtb)
{
	Mat res = Mat::zeros(3, 3, CV_64FC1);
	for (int i = 0; i < 3; i++)
	{
		Mat c = gtb.col(i).t();
		int bestJ = 0;
		double bestNorm = 0;
		for (int j = 0; j < 3; j++)
		{
			double nrm = norm(c * b.col(j));
			if (nrm > bestNorm)
			{
				bestJ = j;
				bestNorm = nrm;
			}
		}
		setRow(res, i, b.col(bestJ));
	}
	return res;
}

void compareWithGroundTruth(string localOut, string reportOut, Mat vpBasis, Mat vpOrthoBasis, Mat groundTruthVpBasis, Mat groundTruthVpOrthoBasis, Mat vpErrorMean, Mat vpErrorStd)
{
	Mat vp = reorderColumnsInVpBasis(vpBasis, groundTruthVpBasis);

}

void RunYorkUrbanDbTest(string dbDir, string reportFilePath, Mat calibrationMatrix, Mat inversedCalibrationMatrix)
{
	ifstream imageDirsList(dbDir + "Manhattan_Image_DB_Names.txt");
	vector<string> imageNames;
	while (!imageDirsList.eof())
	{
		string imageName;
		imageDirsList >> imageName;
		if (imageName.length() > 0 && imageName[imageName.length() - 1] == '\\')
			imageName = imageName.substr(0, imageName.length() - 1);
		if (imageName.length() > 1)
			imageNames.push_back(imageName);
	}

	int n = imageNames.size();
	//n = 10;
	if (n == 0)
		return;

	ofstream globalReport(reportFilePath);

	Mat vpError = Mat::zeros(n, 3, CV_64FC1);
	Mat vpErrorMean = Mat::zeros(1, 3, CV_64FC1), vpErrorStd = Mat::zeros(1, 3, CV_64FC1);
	for (int i = 0; i < n; i++)
	{
		string name = imageNames[i];
		string path = dbDir + name + "/";
		string in = path + name + ".jpg";
		string out = path + name + "_gyrocam_processed.jpg";

		Mat vpBasis, vpOrthoBasis;
		processImage(in, out, calibrationMatrix, inversedCalibrationMatrix, vpBasis, vpOrthoBasis);

		out = path + name;
		saveVanishingPointsDirections(out, vpBasis, vpOrthoBasis);

		in = path + name + "GroundTruthVP_CamParams.mat.txt";
		Mat groundTruthVpBasis = readVpBasis(in);
		in = path + name + "GroundTruthVP_Orthogonal_CamParams.mat.txt";
		Mat groundTruthVpOrthoBasis = readVpBasis(in);
		
		out = path + name + "_gyrocam_compare_vp_basis.txt";
		//compareWithGroundTruth(out, reportFilePath, vpBasis, vpOrthoBasis, groundTruthVpBasis, groundTruthVpOrthoBasis, vpErrorMean, vpErrorStd);

		Mat vp = reorderColumnsInVpBasis(vpOrthoBasis, groundTruthVpOrthoBasis);
		for (int j = 0; j < 3; j++)
			vpError.at<double>(i, j) = acos(norm(vp.row(j) * groundTruthVpOrthoBasis.col(j)));

		vpErrorMean += vpError.row(i);
		ofstream localCompare(out);
		localCompare << vpError.row(i) << endl;
		localCompare.flush();
		localCompare.close();

		globalReport << name << ": " << vpError.row(i) << endl;

		cout << i << endl;
	}

	vpErrorMean /= n;
	globalReport << "err mean = " << vpErrorMean << endl;

	if (n < 2)
		return;

	for (int i = 0; i < n; i++)
	{
		Mat t = vpError.row(i) - vpErrorMean;
		for (int j = 0; j < 3; j++)
			vpErrorStd.at<double>(0, j) += sqrt(norm(t.t() * t));
	}	
	vpErrorStd /= n - 1;

	globalReport << "err std = " << vpErrorStd << endl;
	globalReport.flush();
	globalReport.close();
}

int main(int argc, char** argv)
{
    std::string in = "../../TestSamples/urban3.jpg";
	in = "../../TestSamples/YorkUrbanDB/";
	std::string out = "../../TestSamples/output.jpg";
	out = "../../TestSamples/YorkUrbanDB/report.txt";
	std::string calibrationMatrixPath = "../../TestSamples/YorkUrbanDB/cameraParameters.txt";
	// modes
	bool waitMode = true;
	// required and optional params
	bool inPathIsSet = false;
	bool outPathIsSet = false;
	bool calibMatrixIsSet = false;
	
	for (int i = 1; i < argc; i++)
	{
		std::string s(argv[i]);
		if (s.length() > 0 && s[0] == '-')
		{
			// optional
			std::transform(s.begin(), s.end(), s.begin(), ::tolower);
			if (s == "-nowait")
				waitMode = false;
			else if (s == "-silent")
				SILENT_MODE = true;
			else if (s == "-defaultYorkCalibMat")
				calibMatrixIsSet = true;
			else if (s == "-yorkUrbanDb")
				YORK_URBAN_DB_TEST_MODE = true;
		}
		else if (!inPathIsSet)
		{
			in = s;
			inPathIsSet = true;
		}
		else if (!outPathIsSet)
		{
			out = s;
			outPathIsSet = true;
		}
		else
		{
			calibrationMatrixPath = s;
			calibMatrixIsSet = true;
		}
	}
	
	calibMatrixIsSet = true;
	Mat calibrationMatrix = readCalibrationMatrix(calibMatrixIsSet, calibrationMatrixPath);
	Mat inversedCalibrationMatrix = calibrationMatrix.inv();
	
	//if (YORK_URBAN_DB_TEST_MODE)
	{
		SILENT_MODE = true;
		RunYorkUrbanDbTest(in, out, calibrationMatrix, inversedCalibrationMatrix);
		return 0;
	}
	

	//processImageWithouthCalibration(in, out);	
	Mat vpBasis, vpOrthoBasis;
	processImage(in, out, calibrationMatrix, inversedCalibrationMatrix, vpBasis, vpOrthoBasis);	
	saveVanishingPointsDirections(out, vpBasis, vpOrthoBasis);

	if (waitMode)
		waitKey();

    return 0;
}

//
//Point3d refineVanishingPointWithoutUncalibration(vector<LineSegment> segments)
//{
//	Mat A = Mat::zeros(segments.size(), 3, CV_64FC1);
//	for (int i = 0; i < segments.size(); i++)
//	{
//		Point3d originLine(segments[i].line);
//		originLine /= norm(originLine);
//		setRow(A, i, Mat(originLine));
//	}
//
//	Mat res;
//	SVD::solveZ(A, res);
//	return Point3d(res);
//}
//
//void processImageWithouthCalibration(std::string in, std::string out)
//{
//    Mat image = imread(in, 0);
//	RansacClusterizer clusterizer = initRansacClusterizer(image);
//
//	image = imread(in, 1);
//	vector<Scalar> colors = getColors();
//	Scalar black(0, 0, 0);
//	drawFoundSegments(clusterizer.segments, image, black, 1);
//
//	vector<Point3d> vps;
//	for (int i = 0; i < 3; i++)
//	{
//		if (clusterizer.notUsed.size() == 0)
//			break;
//
//		Point3d originVp;
//		vector<LineSegment> found = clusterizer.nextCluster(originVp);
//		drawFoundSegments(found, image, colors[i] / 2);
//
//		Point3d refinedVp = refineVanishingPointWithoutUncalibration(found);
//		drawFoundSegments(getSegmentsIncidentWithVanishingPoint(refinedVp, clusterizer.segments), image, colors[i]);
//		vps.push_back(refinedVp);
//	}
//
//	imwrite(out, image);
//
//	Mat vpBasis = getRotationMatrixBasedOnVanishingPoints(vps);
//	Mat orthoVpBasis = getNearestOrthogonalMatrix(vpBasis);
//	
//	std::ofstream outfile(out + ".txt");
//	outfile << vpBasis << endl << orthoVpBasis << endl;
//	outfile.flush();
//	outfile.close();
//
//	std::cout << vpBasis << endl;
//	std::cout << orthoVpBasis << endl;
//
//	imshow("result", image);
//}
