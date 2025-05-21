#ifndef _ANGLESOLVER_H
#define _ANGLESOLVER_H

#include "opencv2/core/core.hpp"
#include<iostream>
#include <opencv2/opencv.hpp>

namespace rm 
{

struct AngleSolverParam
{
    cv::Mat CAM_MATRIX;
    cv::Mat DISTORTION_COEFF;
	
	static std::vector<cv::Point3f> POINT_3D_OF_ARMOR_BIG;
	static std::vector<cv::Point3f> POINT_3D_OF_ARMOR_SMALL;
	static std::vector<cv::Point3f> POINT_3D_OF_RUNE;
    double Y_DISTANCE_BETWEEN_GUN_AND_CAM = 0;	//枪口补偿
    cv::Size CAM_SIZE = cv::Size(1280,1024);	//相机画幅大小

    void readFile(const int id);	//相机id
};
class AngleSolver
{
public:
    AngleSolver();
    AngleSolver(const AngleSolverParam& AngleSolverParam);

	enum AngleFlag
	{
		ANGLE_ERROR = 0,                
		ONLY_ANGLES = 1,		
		TOO_FAR = 2,			
		ANGLES_AND_DISTANCE = 3		
	}; 


	void setTarget(const std::vector<cv::Point2f> objectPoints, int objectType);  //set corner points for PNP
	
	AngleFlag solve();

	void compensateOffset();	//偏移补偿
	void compensateGravity();	//重力补偿

	void set_UserType(int usertype);

	void set_EnemyType(int enemytype);

	void set_BulletSpeed(int bulletSpeed);

	const cv::Vec2f get_Angle();

    double get_Distance();

private:
	AngleSolverParam _params;
	cv::Mat _rVec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat _tVec = cv::Mat::zeros(3, 1, CV_64FC1);
	std::vector<cv::Point2f> point_2d_of_armor;
	
	int angle_solver_algorithm = 0;
	cv::Point2f centerPoint;
	std::vector<cv::Point2f> target_nothing;
	double xErr, yErr, euclideanDistance;	//pitch,yaw,距离
	cv::Size2i image_size = cv::Size2i(1280,1024);
	int user_type = 1;
	int enemy_type = 1;
	double bullet_speed = 22000;
	cv::Mat _cam_instant_matrix;
};
}    
