#pragma once

namespace gyrocam
{
	static const double POCKET_IMAGE_WIDTH = 640;
	static const double POCKET_IMAGE_HEIGHT = 480;
	static const double BIG_IMAGE_SCALE = 1.5;

	static const double MIN_ALLOWED_LINE_SEGMENT_LENGTH = 20;
	
	static const double RANSAC_PROBABILITY = 0.95;
	static const double DISTANCE_EPSILON = 4;
	static const double ANGLE_EPSILON = 0.04;
	static const double REFINED_ANGLE_EPSILON = 0.02;

	//static const double pi = 3.14159265358979323846;
	static const double INFINITE_POINT_Z_EPSILON = 1e-12;

	class Settings
	{
	public:		
		bool TRACE_ENABLED;
		bool BUILD_IMAGE;
		bool DRAW_RAW_SEGMENTS;
		bool SHOW_IMAGE;
		bool WAIT_AFTER;
		bool SCALE_ENABLED;
		bool POCKET_SIZE;
		bool YORK_URBAN_DB_TEST_MODE;

		Settings()
		{
			TRACE_ENABLED = false;
			BUILD_IMAGE = true;
			DRAW_RAW_SEGMENTS = true;
			SHOW_IMAGE = true;
			WAIT_AFTER = true;
			SCALE_ENABLED = true;
			POCKET_SIZE = false;
			YORK_URBAN_DB_TEST_MODE = false;
		}
	};
}