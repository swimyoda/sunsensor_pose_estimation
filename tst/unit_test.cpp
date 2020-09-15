#include <iostream>
#include <gtest/gtest.h>
#include "sunsensor/"

class PoseTest : testing::Test
{
	PoseEst* est;
	
	PoseTest(double * s, double* angles, int err0, int err1)
	{
		est = new PoseEst();
	}
	
	~PoseTest()
	{
		delete est;
	}
}

TEST_F(PoseTest, edge) {

	ASSERT_EQ(0,0);
}



int main(int argc, char **argv) {

	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
