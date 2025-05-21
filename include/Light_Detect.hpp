#ifndef LIGHT_DETECT
#define LIGHT_DETECT

#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>

namespace rm
{
enum ColorChannels
{
	BLUE = 0,
	RED = 1
};
struct Light_Param
{
    float light_min_area;
    float light_max_area;
    float light_min_size;
    float light_max_angle;
    float light_contour_min_solidity;
    float light_max_ratio;

	float light_max_angle_diff_;
	float light_max_height_diff_ratio_;
	float light_max_y_diff_ratio_;
	float light_min_x_diff_ratio_;
    
    Light_Param()
    {
        light_min_area = 10;
        light_max_angle = 45.0;
        light_min_size = 5.0;
        light_contour_min_solidity = 0.5;
        light_max_ratio = 1.0;

		light_max_angle_diff_=7.0;
		light_max_height_diff_ratio_=0.2;
		light_max_y_diff_ratio_=2.0;
		light_min_x_diff_ratio_=0.5;
    }
};

class LightDescription
{
public:
    LightDescription(){}
    LightDescription(const cv::RotatedRect& light)
    {
        width = light.size.width;
        length = light.size.height;
        center = light.center;
        angle = light.angle;
        area = light.size.area();
		light.points(points);

	}
    cv::RotatedRect rec() const
	{
		return cv::RotatedRect(center, cv::Size2f(width, length), angle);
	}
	
public:
	float width;
	float length;
	cv::Point2f center;
	float angle;
	float area;
	cv::Point2f points[4];
};


}
#endif