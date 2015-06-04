#include "TestTools.h"
#include "Geometry.h"
#include <iostream>

#include "opencv2/imgproc/imgproc.hpp"

namespace gyrocam
{
	void testOut(cv::Vec4i s)
	{
		std::cout << "[(" << s[0] << ", " << s[1] << "), (" << s[2] << ", " << s[3] << ")]";
	}

	void testOut(cv::Point3d p)
	{
		std::cout << p;
	}
	
	std::vector<cv::Scalar> getColors()
	{
		std::vector<cv::Scalar> colors;
		cv::Scalar blue(255, 0, 0);
		cv::Scalar green(0, 255, 0);
		cv::Scalar red(0, 0, 255);
		cv::Scalar black(0, 0, 0);
	
		colors.push_back(black);
		colors.push_back(blue);
		colors.push_back(green);
		colors.push_back(red);
		colors.push_back(blue + green);
		colors.push_back(blue + red);
		colors.push_back(green + red);
		return colors;
	}
	
	void drawFoundSegments(const std::vector<LineSegment> &segments, const cv::Mat &image, const cv::Scalar &color, int thickness)
	{
		for (int i = 0; i < segments.size(); i++)
		{
			cv::Vec4i origin = segments[i].origin;
			line(image, cv::Point(origin[0], origin[1]), cv::Point(origin[2], origin[3]), color, thickness);
		}
	}
	void drawPointMarkerIfVisibleOnImage(const cv::Point3d &p, const cv::Mat &image, const cv::Scalar &color)
	{
		if (p.x < 0 || p.x > image.cols || p.y < 0 || p.y > image.rows)
			return;

		double sq2 = sqrt(2.0);
		double r = 10;
		double d = r*sq2/2;

		circle(image, cv::Point2d(p.x, p.y), r, color, 2);
		line(image, cv::Point2d(p.x-d, p.y+d), cv::Point2d(p.x+d, p.y-d), color, 2);
		line(image, cv::Point2d(p.x-d, p.y-d), cv::Point2d(p.x+d, p.y+d), color, 2);
	}
	std::string getEulerAnglesString(const cv::Mat &ea)
	{
		std::stringstream stream;
		stream << "roll = " << ea.at<double>(0, 0) << "; ";
		stream << "pitch = " << ea.at<double>(0, 1) << "; ";
		stream << "yaw = " << ea.at<double>(0, 2) << "; ";
		return stream.str();
	}
	void drawEulerAngles(cv::Mat &image, const cv::Mat &ea)
	{
		std::string s = getEulerAnglesString(ea);		
		putText(image, s, cv::Point(20, 20), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
	}
	void drawAxes(cv::Mat &image, const cv::Mat &vps)
	{
		cv::Point2d o(image.cols/2, image.rows/2);
		cv::Mat e(vps);
		e.row(1) = -1 * e.row(1);
		e.row(2) = -1 * e.row(2);
		for (int i = 0; i < 3; i++)
		{
			cv::Point3d p(e.col(i));
			normalizeZ(p);
			cv::Point2d _p(p.x, p.y);

			cv::line(image, o, o + _p, cv::Scalar::all(255), 1);
		}
	}

	std::string getBaseIdentificationPath(const std::string &path)
	{
		int extInd = path.find_last_of('.');
		return (extInd == std::string::npos || (extInd + 1 < path.length() && path[extInd + 1] == '/')) 
			? path 
			: path.substr(0, extInd);
	}

	std::string getExtension(const std::string &path)
	{
		auto idPart = getBaseIdentificationPath(path);
		return path.substr(idPart.length(), path.length() - idPart.length());
	}

	SingleRunConfig::SingleRunConfig() {}
	SingleRunConfig::SingleRunConfig(const std::string &inPath, const std::string &calibrationMatrixPath)
	{
		originPath = inPath;
		this->calibrationMatrixPath = calibrationMatrixPath;
		identificationPathPart = getBaseIdentificationPath(originPath);
		extension = getExtension(originPath);
	}

	std::string SingleRunConfig::getInputImagePath() { return originPath; }
	std::string SingleRunConfig::getCalibrationMatrixPath() { return calibrationMatrixPath; }
	std::string SingleRunConfig::getOutputImagePath() { return identificationPathPart + "_gyrocam_processed" + extension; }
	std::string SingleRunConfig::getOutputNonOrthogonalVpsPath() { return identificationPathPart + "_gyrocam_vp_basis.txt"; }
	std::string SingleRunConfig::getOutputOrthogonalVpsPath() { return identificationPathPart + "_gyrocam_vp_ortho_basis.txt"; }
	std::string SingleRunConfig::getOutputAnglesPath() { return identificationPathPart + "_gyrocam_angles.txt"; }

	
	YorkUrbanDbTestRunConfig::YorkUrbanDbTestRunConfig() {}
	YorkUrbanDbTestRunConfig::YorkUrbanDbTestRunConfig(const std::string &inPath) { originPath = inPath[inPath.length() - 1] == '/' ? inPath : inPath + "/"; }

	std::string YorkUrbanDbTestRunConfig::getImageNamesListPath() { return originPath + "Manhattan_Image_DB_Names.txt"; }
	std::string YorkUrbanDbTestRunConfig::getCalibrationMatrixPath() { return originPath + "cameraParameters.txt"; }
	std::string YorkUrbanDbTestRunConfig::getSingleRunBasePath(const std::string &imageFolder) { return originPath + imageFolder + "/" + imageFolder; }
	std::string YorkUrbanDbTestRunConfig::getSingleRunImagePath(const std::string &basePath) { return basePath + ".jpg"; }
	std::string YorkUrbanDbTestRunConfig::getOutputSingleReportPath(const std::string &basePath) { return basePath + "_gyrocam_compare_vp_basis.txt"; }
	std::string YorkUrbanDbTestRunConfig::getSingleRunGroundTruthPath(const std::string &basePath) { return basePath + "GroundTruthVP_CamParams.mat.txt"; } 
	std::string YorkUrbanDbTestRunConfig::getSingleRunOrthogonalGroundTruthPath(const std::string &basePath) { return basePath + "GroundTruthVP_Orthogonal_CamParams.mat.txt"; } 
	std::string YorkUrbanDbTestRunConfig::getOutputGlobalReportPath() { return originPath + "/gyrocam_report.txt"; }
}