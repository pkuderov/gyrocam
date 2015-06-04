#pragma once
#include <string>
#include "opencv2/core/core.hpp"

namespace gyrocam
{
	cv::Mat readCalibrationMatrix(const std::string &path);
	void saveMatrix(const std::string &path, const cv::Mat &vps);
	std::vector<std::string> extractListFromFile(const std::string &path);
	cv::Mat readVpBasis(const std::string &path);
}