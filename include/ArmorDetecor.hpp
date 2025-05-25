#ifndef ARMOR_DETECOR
#define ARMOR_DETECOR

#include<opencv2/opencv.hpp>
#include<array>
#include<opencv2/ml.hpp>

//尝试上传更改
namespace rm
{
enum ColorChannels
{
	BLUE = 0,
	RED = 1
};
enum ARMORTYPE
{	
	UNKNOWN_ARMOR = 2,
	SMALL_ARMOR = 0,
	BIG_ARMOR = 1,
};
struct ArmorParam
{
    int brightness_threshold;
    int color_threshold;
    float light_color_detect_extend_ratio;

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

    float armor_big_armor_ratio;
    float armor_small_armor_ratio;
	float armor_min_aspect_ratio_;
	float armor_max_aspect_ratio_;

    int enemy_color;

    ArmorParam()
    {
        brightness_threshold = 100;
        color_threshold = 40;
        light_color_detect_extend_ratio = 1.1;

        light_min_area = 10;
        light_max_angle = 45.0;
        light_min_size = 5.0;
        light_contour_min_solidity = 0.5;
        light_max_ratio = 1.0;

		light_max_angle_diff_=7.0;
		light_max_height_diff_ratio_=0.2;
		light_max_y_diff_ratio_=2.0;
		light_min_x_diff_ratio_=0.5;

        armor_big_armor_ratio = 3.2;
        armor_small_armor_ratio = 2;
		armor_min_aspect_ratio_=1.0;
		armor_max_aspect_ratio_=5.0;

        enemy_color = RED;
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


class ArmorDescription
{
public:
	
	ArmorDescription();

	
	ArmorDescription( const LightDescription& l_Light, const LightDescription& r_Light, const int armorType, const cv::Mat& srcImg,ArmorParam param);
	
	void clear()
	{
		distScore = 0;
		for(int i = 0; i < 4; i++)
		{
			vertex[i] = cv::Point2f(0, 0);
		}
		type = UNKNOWN_ARMOR;
	}

	std::array<cv::RotatedRect, 2> lightPairs; //0 left, 1 right
	float distScore;		//S2 = e^(-offset)

	
	std::vector<cv::Point2f> vertex; //four vertex of armor area, lihgt bar area exclued!!	
	
	cv::Point2f center;
	//	0 -> small
	//	1 -> big
	//	-1 -> unkown
	int type;
};


class ArmorDetector
{
public:
	
	enum ArmorFlag
	{
		ARMOR_NO = 0,		// not found
		ARMOR_LOST = 1,		// lose tracking
		ARMOR_GLOBAL = 2,	// armor found globally
		ARMOR_LOCAL = 3		// armor found locally(in tracking mode)
	};

    ArmorDetector();
	ArmorDetector(const ArmorParam& armorParam);

	void init(const ArmorParam& armorParam);

	void setEnemyColor(ColorChannels enemy_color);

	int detect();

	const std::vector<cv::Point2f> getArmorVertex() const;

    int getArmorType() const;


private:
	ArmorParam param;
	int enemy_color;
	int self_color;

	cv::Mat srcImg; //source img
	cv::Mat grayImg; //gray img of roi

	std::vector<ArmorDescription> armors;

	ArmorDescription targetArmor; //relative coordinates

	int flag;
	
};

}

#endif