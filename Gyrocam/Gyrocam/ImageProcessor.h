#pragma once
#include <set>
#include "Config.h"
#include "TestTools.h"

namespace gyrocam
{
	class ImageProcessor
	{
		SingleRunConfig config;
		Settings settings;

		cv::Mat calibrationMatrix, inversedCalibrationMatrix;
		cv::Mat image;

		std::vector<LineSegment> segments;
		std::set<int> notUsedSegments;
		std::vector<cv::Scalar> colors;

		void initColors();
		void loadCalibrationMatrices();
		void loadImage();
		void extractLineSegments();
		void findVanishingPoint();
		
		cv::Point3d refineVanishingPoint(const std::vector<LineSegment> &cluster);		
		cv::Point3d smartVpRefinement(const std::vector<LineSegment> &originCluster, int i);
		cv::Mat toNormalized(const cv::Mat &line);
		cv::Mat toNormalized(const LineSegment &s);
		cv::Point3d fromNormalized(const cv::Mat &line);		
		cv::Mat getRotationMatrix(const std::vector<cv::Point3d> &vps);
		
		cv::Point3d findVanishingPoint(std::vector<cv::Point3d> &vps);
		void refineVpBasis(cv::Mat &vpBasis);
		void solveZ(const cv::Mat &a, cv::Mat &res);
		
		void drawInducedCluster(const cv::Point3d &vp, const std::vector<LineSegment> &cluster, const cv::Scalar &color, bool isDrawNeeded);
		void drawProcessedImage(std::string title);
		void saveResults(const SingleRunResult &result);

	public:
		ImageProcessor(const SingleRunConfig &config, const Settings &settings);
		void loadConfig(const SingleRunConfig &config);
		SingleRunResult process();
		~ImageProcessor(void);
	};
}
