#include "IoProvider.h"
#include <iostream>
#include <fstream>

namespace gyrocam
{	
	cv::Mat readCalibrationMatrix(const std::string &path)
	{
		cv::Mat calibrationMatrix = cv::Mat::eye(3, 3, CV_64FC1);
		if (path.length() == 0)
			return calibrationMatrix;
	
		double focalLength, pixelSize, ppHor, ppVer;

		std::string _;
		std::ifstream in(path);
		in >> _ >> focalLength;
		in >> _ >> pixelSize;
		in >> _ >> ppHor >> ppVer;
		in.close();

		calibrationMatrix.at<double>(0, 0) = focalLength / pixelSize;
		calibrationMatrix.at<double>(1, 1) = focalLength / pixelSize;
		calibrationMatrix.at<double>(0, 2) = ppHor;
		calibrationMatrix.at<double>(1, 2) = ppVer;
		return calibrationMatrix;
	}

	void saveMatrix(const std::string &path, const cv::Mat &vps)
	{	
		std::ofstream out(path);
		out << vps << std::endl;
		out.flush();
		out.close();
	}
	
	std::vector<std::string> extractListFromFile(const std::string &path)
	{
		// 'XXXX\' or 'XXXX' format
		std::ifstream in(path);
		std::vector<std::string> imageNames;
		while (!in.eof())
		{
			std::string name;
			in >> name;
			if (name.length() > 0 && name[name.length() - 1] == '\\')
				name = name.substr(0, name.length() - 1);
			if (name.length() > 1)
				imageNames.push_back(name);
		}
		return imageNames;
	}

	cv::Mat readVpBasis(const std::string &path)
	{
		std::ifstream in(path);
		std::string _;
		getline(in, _);

		double t;
		cv::Mat res = cv::Mat::zeros(3, 3, CV_64FC1);
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				in >> t;
				res.at<double>(i, j) = t;
			}
		}

		return res;
	}

}