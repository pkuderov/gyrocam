#pragma once
#include "TestTools.h"

namespace gyrocam
{
	class YorkUrbanDbTester
	{
		YorkUrbanDbTestRunConfig config;
		Settings settings;

	public:
		YorkUrbanDbTester(const YorkUrbanDbTestRunConfig &config, const Settings &settings);
		void run(int k);
	};
}