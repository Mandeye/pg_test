#include "gtest/gtest.h"

#include <iostream>     // std::string
#include <fstream>      // std::ifstream
#include "ladybugGeometry.h"

LadybugGeom myGeometry;
#define ERR 0.1


void testInner(int cam, double x, double y, double z, double eu, double ev)
{
	double u = 0;
	double v = 0;
	myGeometry.myladybugXYZtoRC(x,y,z,cam,&u,&v);
	EXPECT_NEAR(u, eu , ERR);
	EXPECT_NEAR(v, ev , ERR);
}

#include "tests.inc"

int main(int argc, char **argv)
{

  ::testing::InitGoogleTest(&argc, argv);
#include "camera_config.inc"
  // set camera params

  return RUN_ALL_TESTS();
}
