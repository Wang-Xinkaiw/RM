#include "ArmorDetecor.hpp"
#include "AngleSolver_test.hpp"
using namespace std;
using namespace cv;
using namespace rm;
#define TEST_ARMOR_DETECOR

int main()
{

rm::ArmorParam param;

rm::ArmorDetector test;
test.init(param);

test.setEnemyColor(rm::RED);
//ÊÓÆµÁ÷
cv::VideoCapture cap("/home/bill/Downloads/test_video/Video_20250111143817266.avi");
if (!cap.isOpened())
{
	std::cout << "Error opening video file" << std::endl;
	return -1;
}
cv::Mat srcImg;

while (cap.isOpened())
{
	cap >> srcImg;
	if (srcImg.empty())
	{
		std::cout << "End of video" << std::endl;
		break;
	}
}
test.detect();
//imshow("FirstResult",FirstResult); 
//test.~ArmorDetector();
return 0;
}

//angleParam.readFile(1);//only 0 1 2 
//AngleSolver angleSolver;
//AngleSolver();
//angleSolver.init(angleParam);
//
//for (auto& armor : armors)
//{
//	angleSolver.set_EnemyType(1);
//	angleSolver.set_Target(armor.vertex, 1);
//	cout << armor.vertex << endl;
//	angleSolver.solve();
//}
//
//cout << angleSolver.get_Angle() << endl;
//cout << angleSolver.get_Distance() << endl;
//waitKey(10);
//showLights(srcImg, lightInfos);
//showArmors(srcImg, armors, targetArmor);
//if (!lightInfos.empty())
//{
//	lightInfos.clear();
//
//}
//
//if (!armors.empty())
//{
//	armors.clear();
//	targetArmor.clear();
//}
//
//
//srcImg.release();
//destroyAllWindows();