#pragma once
#include "opencv2/core/core.hpp"
#include <vector>
#include "Config.h"
#include "LineSegment.h"

namespace gyrocam
{
	std::vector<cv::Scalar> getColors();
	void drawFoundSegments(const std::vector<LineSegment> &segments, const cv::Mat &image, const cv::Scalar &color, int thickness = 2);
	void drawPointMarkerIfVisibleOnImage(const cv::Point3d &p, const cv::Mat &image, const cv::Scalar &color);
	std::string getEulerAnglesString(const cv::Mat &ea);
	void drawEulerAngles(cv::Mat &image, const cv::Mat &ea);
	void drawAxes(cv::Mat &image, const cv::Mat &vps);
	
	std::string getBaseIdentificationPath(const std::string &path);
	std::string getExtension(const std::string &path);


	class SingleRunConfig
	{
		std::string originPath, calibrationMatrixPath, identificationPathPart, extension;
	
	public:
		SingleRunConfig();
		SingleRunConfig(const std::string &inPath, const std::string &calibrationMatrixPath);

		std::string getInputImagePath();
		std::string getCalibrationMatrixPath();
		std::string getOutputImagePath();
		std::string getOutputNonOrthogonalVpsPath();
		std::string getOutputOrthogonalVpsPath();
		std::string getOutputAnglesPath();
	};
	
	class YorkUrbanDbTestRunConfig
	{
		std::string originPath;
	
	public:
		YorkUrbanDbTestRunConfig();
		YorkUrbanDbTestRunConfig(const std::string &inPath);

		std::string getImageNamesListPath();
		std::string getCalibrationMatrixPath();
		std::string getSingleRunBasePath(const std::string &imageFolder);
		std::string getSingleRunImagePath(const std::string &basePath);
		std::string getOutputSingleReportPath(const std::string &basePath);
		std::string getSingleRunGroundTruthPath(const std::string &basePath);
		std::string getSingleRunOrthogonalGroundTruthPath(const std::string &basePath);
		std::string getOutputGlobalReportPath();
	};

	class SingleRunResult
	{
	public:
		cv::Mat vpBasis, orthoVpBasis, eulerAngles;
		double runTime;
	};

	class BatchRunResult
	{

	};
}

