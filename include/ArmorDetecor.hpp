#ifndef ARMOR_DETECOR_H
#define ARMOR_DETECOR_H

#include<opencv2/opencv.hpp>
#include<array>
#include<opencv2/ml.hpp>


namespace rm
{

enum ColorChannels
{
	BLUE = 0,
	RED = 1
};

enum ARMORTYPE
{	
	SMALL_ARMOR = 0,
	BIG_ARMOR = 1,
	UNKNOWN_ARMOR = 2
};

struct ArmorParam
{
	//预处理所需参数：亮度，颜色阈值；灯条扩大比例
    int brightness_threshold;
    int color_threshold;
    float light_color_detect_extend_ratio;

	//灯条筛选参数：灯条最小，大面积；灯条最小尺寸；最大角度；凸度；最大比例
    float light_min_area;
    float light_max_area;
    float light_min_size;
    float light_max_angle;
    float light_contour_min_solidity;
    float light_max_ratio;

	//灯条对筛选参数：最大角度差；高度差比率（高/最大的长度）；最大y方向差比率；最小x方向比率
	float light_max_angle_diff_;
	float light_max_height_diff_ratio_;
	float light_max_y_diff_ratio_;
	float light_min_x_diff_ratio_;

	//装甲板筛选参数：大，小装甲板比例；最小，最大方面比率
    float armor_big_armor_ratio;
    float armor_small_armor_ratio;
	float armor_min_aspect_ratio_;
	float armor_max_aspect_ratio_;

	//其他参数：敌人颜色
    int enemy_color;

	//初始化参数
    ArmorParam()
    {
        brightness_threshold = 160;
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
    LightDescription(){}；
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
		//numScore = 0;
		distScore = 0;
		for(int i = 0; i < 4; i++)
		{
			vertex[i] = cv::Point2f(0, 0);
		}
		type = UNKNOWN_ARMOR;
	}

	//void getLastImg(const cv::Mat& lastImg);

	std::array<cv::RotatedRect, 2> lightPairs; //0 left, 1 right
	float distScore;		//S2 = e^(-offset)

	
	std::vector<cv::Point2f> vertex; //four vertex of armor area, lihgt bar area exclued!!	
	
	cv::Point2f center;
	cv::Mat lastImg;
	int type;
};


class ArmorDetector
{
public:
	
	enum ArmorFlag
	{
		ARMOR_NO = 0,		// not found
		ARMOR_LOST = 1,		// lose tracking
		ARMOR_NEW = 2,	// armor found new
		ARMOR_LOCAL = 3		// armor found locally(in tracking mode)
	};

    ArmorDetector(const ArmorParam& armorParam);
	ArmorDetector(const ArmorDetector *armorDetector);
	~ArmorDetector(){}

	void setEnemyColor(int enemy_color);

	int detect();

	const std::vector<cv::Point2f> getArmorVertex() const;

    int getArmorType() const;


private:
	ArmorParam param;
	int enemy_color;
	int self_color;

	cv::Mat srcImg;
	cv::Mat grayImg; 
	cv::Mat roiImg;

	std::vector<ArmorDescription> armors;

	ArmorDescription targetArmor; 

	int flag;
	// bool trackingMode;
};

}

#endif