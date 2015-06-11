#include <iostream>
#include <fstream>
#include <string>
#include <time.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "Config.h"
#include "RansacClusterizer.h"

#include "TestTools.h"
#include "Geometry.h"
#include "Shared.h"
#include "ImageProcessor.h"
#include "IoProvider.h"

using namespace gyrocam;

std::string inPath, calibrationMatrixPath;
Settings settings;

class YorkUrbanDbTester
{
	YorkUrbanDbTestRunConfig config;
	Settings settings;

public:
	YorkUrbanDbTester(const YorkUrbanDbTestRunConfig &config, const Settings &settings)
	{
		this->config = config;
		this->settings = settings;
	}

	void run()
	{
		std::vector<std::string> imageNames = extractListFromFile(config.getImageNamesListPath());
		int n = imageNames.size();
		std::cout << "n: " << n << std::endl;
		//n = 10;
		if (n == 0)
			return;

		std::ofstream globalReport(config.getOutputGlobalReportPath());
		
		SingleRunConfig singleRunConfig(
			config.getSingleRunImagePath(config.getSingleRunBasePath(imageNames[0])), 
			config.getCalibrationMatrixPath()
		);
		ImageProcessor imageProcessor(singleRunConfig, settings);

		cv::Mat vpError = cv::Mat::zeros(n, 3, CV_64FC1);
		cv::Mat vpErrorMean = cv::Mat::zeros(1, 3, CV_64FC1), vpErrorStd = cv::Mat::zeros(1, 3, CV_64FC1);
		double elapsedTime = 0;
		for (int i = 0; i < n; i++)
		{
			std::string basePath = config.getSingleRunBasePath(imageNames[i]);
			
			SingleRunConfig singleRunConfig(config.getSingleRunImagePath(basePath), config.getCalibrationMatrixPath());
			imageProcessor.loadConfig(singleRunConfig);
			SingleRunResult singleRunResult = imageProcessor.process();
			elapsedTime += singleRunResult.runTime;

			cv::Mat groundTruthVpOrthoBasis = readVpBasis(config.getSingleRunOrthogonalGroundTruthPath(basePath));
		
			//cout << singleRunResult.orthoVpBasis << std::endl;
			groundTruthVpOrthoBasis.row(1) = -1 * groundTruthVpOrthoBasis.row(1);
			reorderColumn(groundTruthVpOrthoBasis, 1);
			reorderColumn(groundTruthVpOrthoBasis, 0);
			reorderColumn(groundTruthVpOrthoBasis);
			
			cv::Mat gtvpT = groundTruthVpOrthoBasis.t();
			
			double angle = angleBetween(gtvpT.row(0), singleRunResult.orthoVpBasis.col(0));
			if (abs(CV_PI/2 - abs(angle - CV_PI/2)) > 1.2)
			{
				swapColumns(groundTruthVpOrthoBasis, 0, 2);
				gtvpT = groundTruthVpOrthoBasis.t();
			}

			for (int j = 0; j < 3; j++)
				vpError.at<double>(i, j) = angleBetweenAbs(gtvpT.row(j), singleRunResult.orthoVpBasis.col(j));

			vpErrorMean += vpError.row(i);
			std::ofstream localCompare(config.getOutputSingleReportPath(basePath));
			localCompare << vpError.row(i) << std::endl;
			localCompare.flush();
			localCompare.close();

			globalReport << imageNames[i] << ": " << vpError.row(i) << std::endl;

			std::cout << i << std::endl;
		}

		vpErrorMean /= n;
		globalReport << "err mean = " << vpErrorMean << std::endl;

		if (n < 2)
			return;

		for (int i = 0; i < n; i++)
		{
			cv::Mat t = vpError.row(i) - vpErrorMean;
			for (int j = 0; j < 3; j++)
			{
				double tt = t.at<double>(0, j);
				vpErrorStd.at<double>(0, j) += tt * tt;
			}
		}	
		for (int i = 0; i < 3; i++)
			vpErrorStd.at<double>(0, i) = sqrt(vpErrorStd.at<double>(0, i))/(n - 1);

		globalReport << "err std = " << vpErrorStd << std::endl;
		globalReport.flush();
		globalReport.close();

		elapsedTime /= 1000;
		std::cout << "elapsed = " << elapsedTime << "; fps = " << n / elapsedTime << std::endl;
	}
};

void setSingleImageTestSettings()
{
	inPath = "../../TestSamples/urban3.jpg";
	//calibrationMatrixPath = "../../TestSamples/YorkUrbanDB/cameraParameters.txt";
}

void setBatchImageTestSettings()
{
	inPath = "../../TestSamples/bad_indoor/list.txt";
	calibrationMatrixPath = "../../TestSamples/YorkUrbanDB/cameraParameters.txt";
}

void setYorkUrbanDbTestSettings()
{
	settings.YORK_URBAN_DB_TEST_MODE = true;
	settings.SHOW_IMAGE = false;

	inPath = "../../TestSamples/YorkUrbanDB_indoor/";
	calibrationMatrixPath = "../../TestSamples/YorkUrbanDB/cameraParameters.txt";
}

void resolveInputArguments(int argc, char** argv)
{
	for (int i = 1; i < argc; i++)
	{
		std::string s(argv[i]);
		if (s.length() > 0 && s[0] == '-')
		{
			// optional
			std::transform(s.begin(), s.end(), s.begin(), ::tolower);
			if (s == "-trace")
				settings.TRACE_ENABLED = true;
			else if (s == "-noimage")
				settings.BUILD_IMAGE = false;
			else if (s == "-noraw")
				settings.DRAW_RAW_SEGMENTS = false;
			else if (s == "-noshow")
				settings.SHOW_IMAGE = false;
			else if (s == "-nowait")
				settings.WAIT_AFTER = false;
			else if (s == "-yorkurbandb")
			{
				settings.YORK_URBAN_DB_TEST_MODE = true;
				settings.WAIT_AFTER = false;
				settings.SHOW_IMAGE = false;
			}
			else if (s == "-size640")
			{
				settings.POCKET_SIZE = true;
				settings.SCALE_ENABLED = true;
			}
			else if (s == "-size1280")
			{
				settings.POCKET_SIZE = false;
				settings.SCALE_ENABLED = true;
			}
		}
		else if (inPath.empty())
			inPath = s;
		else
			calibrationMatrixPath = s;
	}
}

int main(int argc, char** argv)
{
	// init rand()
	srand (time(NULL));
	//setSingleImageTestSettings();
	//setBatchImageTestSettings();
	//setYorkUrbanDbTestSettings();
	resolveInputArguments(argc, argv);

	if (settings.YORK_URBAN_DB_TEST_MODE)
	{
		// db test
		YorkUrbanDbTestRunConfig config(inPath);
		YorkUrbanDbTester tester(config, settings);
		tester.run();
	}
	//else if (getExtension(inPath) == ".txt")		
	//{
	//	// folder of images
	//	std::vector<std::string> files = extractListFromFile(inPath);
	//	if (files.size() > 0)
	//	{
	//		SingleRunConfig config(files[0], calibrationMatrixPath);
	//		ImageProcessor imageProcessor(config, settings);
	//		
	//		for (int i = 0; i < files.size(); i++)
	//		{
	//			SingleRunConfig config(files[i], calibrationMatrixPath);
	//			imageProcessor.loadConfig(config);
	//			imageProcessor.process();
	//		}
	//	}
	//}
	else
	{
		// single image
		SingleRunConfig config(inPath, calibrationMatrixPath);
		ImageProcessor imageProcessor(config, settings);
		imageProcessor.process();
	}

	/*if (YORK_URBAN_DB_TEST_MODE)
	{
		SILENT_MODE = true;
		RunYorkUrbanDbTest(in, out, calibrationMatrix, inversedCalibrationMatrix);
		return 0;
	}*/
	
	if (settings.WAIT_AFTER)
		cv::waitKey();

    return 0;
}