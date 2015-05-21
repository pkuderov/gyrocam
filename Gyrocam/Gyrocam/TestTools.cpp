#include "TestTools.h"
#include <iostream>

using namespace std;

namespace gyrocam
{
	void testOut(Vec4i s)
	{
		std::cout << "[(" << s[0] << ", " << s[1] << "), (" << s[2] << ", " << s[3] << ")]";
	}

	void testOut(Point3f p)
	{
		std::cout << p;
	}
}