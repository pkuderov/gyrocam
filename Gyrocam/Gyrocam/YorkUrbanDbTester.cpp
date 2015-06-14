#include "YorkUrbanDbTester.h"
#include "IoProvider.h"
#include "ImageProcessor.h"
#include "Shared.h"

#include <iostream>
#include <fstream>

namespace gyrocam
{	
	YorkUrbanDbTester::YorkUrbanDbTester(const YorkUrbanDbTestRunConfig &config, const Settings &settings)
	{
		this->config = config;
		this->settings = settings;
	}

	
	void YorkUrbanDbTester::run(int k)
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
			
			cv::Mat groundTruthVpOrthoBasis = readVpBasis(config.getSingleRunOrthogonalGroundTruthPath(basePath));
			//cout << singleRunResult.orthoVpBasis << std::endl;
			groundTruthVpOrthoBasis.row(1) = -1 * groundTruthVpOrthoBasis.row(1);
			reorderColumn(groundTruthVpOrthoBasis, 1);
			reorderColumn(groundTruthVpOrthoBasis, 0);
			reorderColumn(groundTruthVpOrthoBasis);

			cv::Mat localVpError = cv::Mat::zeros(k, 3, CV_64FC1), localVpErrorMean = cv::Mat::zeros(1, 3, CV_64FC1), localVpErrorStd = cv::Mat::zeros(1, 3, CV_64FC1);
			for (int ii = 0; ii < k; ii++)
			{
				SingleRunResult singleRunResult = imageProcessor.process();
				elapsedTime += singleRunResult.runTime;
				
				cv::Mat gtvpT = groundTruthVpOrthoBasis.t();			
				bool needSwap = angleBetweenAbs(gtvpT.row(0), singleRunResult.orthoVpBasis.col(0)) > angleBetweenAbs(gtvpT.row(2), singleRunResult.orthoVpBasis.col(0));
				if (needSwap)
				{
					swapColumns(groundTruthVpOrthoBasis, 0, 2);
					gtvpT = groundTruthVpOrthoBasis.t();
				}
				
				for (int j = 0; j < 3; j++)
					localVpError.at<double>(ii, j) = angleBetweenAbs(gtvpT.row(j), singleRunResult.orthoVpBasis.col(j));
				
				localVpErrorMean += localVpError.row(ii);
			}
			localVpErrorMean /= k;
			vpError.row(i) += localVpErrorMean;
			vpErrorMean += localVpErrorMean;

			if (k > 1)
			{
				for (int ii = 0; ii < k; ii++)
				{
					cv::Mat t = localVpError.row(ii) - localVpErrorMean;
					for (int j = 0; j < 3; j++)
					{
						double tt = t.at<double>(0, j);
						localVpErrorStd.at<double>(0, j) += tt * tt;
					}
				}
				for (int j = 0; j < 3; j++)
					localVpErrorStd.at<double>(0, j) = sqrt(localVpErrorStd.at<double>(0, j))/(k - 1);
				vpErrorStd += localVpErrorStd;
			}
			
			std::ofstream localCompare(config.getOutputSingleReportPath(basePath));
			localCompare << vpError.row(i) << std::endl;
			localCompare.flush();
			localCompare.close();

			globalReport << imageNames[i] << ": " << vpError.row(i) << std::endl;
			std::cout << i << std::endl;
		}

		vpErrorMean /= n;
		globalReport << "err mean = " << vpErrorMean << std::endl;

		vpErrorStd /= n;
		globalReport << "err std = " << vpErrorStd << std::endl;
		globalReport.flush();
		globalReport.close();

		elapsedTime /= k;
		elapsedTime /= 1000;
		std::cout << "elapsed = " << elapsedTime << "; fps = " << n / elapsedTime << std::endl;
	}
}