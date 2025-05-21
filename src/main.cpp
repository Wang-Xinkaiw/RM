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
test.detect();
//imshow("FirstResult",FirstResult); 
//test.~ArmorDetector();
return 0;
}

