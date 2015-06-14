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
#include "YorkUrbanDbTester.h"

using namespace gyrocam;

std::string inPath, calibrationMatrixPath;
Settings settings;
int TEST_ITERATIONS = 1;

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
		else if (calibrationMatrixPath.empty())
			calibrationMatrixPath = s;
		else
			sscanf(s.c_str(), "%d", &TEST_ITERATIONS);
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
		tester.run(TEST_ITERATIONS);
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