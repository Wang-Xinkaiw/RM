#include "AngleSolver.hpp"
#include "opencv2/opencv.hpp"
#include "math.h"
#include<iostream>

using namespace cv;
using namespace std;

namespace rm
{

//世界坐标
std::vector<cv::Point3f> AngleSolverParam::POINT_3D_OF_ARMOR_BIG = std::vector<cv::Point3f>
{

		cv::Point3f(-105, -30, 0),	//tl
		cv::Point3f(105, -30, 0),	//tr
		cv::Point3f(105, 30, 0),	//br
		cv::Point3f(-105, 30, 0)	//bl
};
std::vector<cv::Point3f> AngleSolverParam::POINT_3D_OF_ARMOR_SMALL = std::vector<cv::Point3f>
{
	cv::Point3f(-65, -35, 0),	//tl
	cv::Point3f(65, -35, 0),	//tr
	cv::Point3f(65, 35, 0),		//br
	cv::Point3f(-65, 35, 0)		//bl
};


AngleSolver::AngleSolver(const AngleSolverParam& AngleSolverParam)
{
	_params = AngleSolverParam;
	_cam_instant_matrix = _params.CAM_MATRIX.clone();
	for (int ll = 0; ll <= 3; ll++)
		target_nothing.push_back(cv::Point2f(0.0, 0.0));
}

AngleSolver::AngleSolver(const AngleSolver *angleLastSetting)
{
	_params = angleLastSetting->_params;
	_cam_instant_matrix = _params.CAM_MATRIX.clone();
	for(int ll = 0; ll <= 3; ll++)
		target_nothing.push_back(cv::Point2f(0.0, 0.0));
}

void AngleSolver::setTarget(const std::vector<cv::Point2f> objectPoints, int objectType)
{
	//objectype 0:小装甲板
	if(objectType == 0 || objectType == 1)
	{
		if(angle_solver_algorithm == 0 || angle_solver_algorithm == 2)
		{
			angle_solver_algorithm = 1; cout << "algorithm is reset to PNP Solution" << endl;
		}
		point_2d_of_armor = objectPoints;
		if(objectType == 0)
			enemy_type = 0;
		else
			enemy_type = 1;
		return;
	}

}

AngleSolver::AngleFlag AngleSolver::solve()
{
	//PnP解算
	if(angle_solver_algorithm == 1)
	{
		if(enemy_type == 1)
			solvePnP(_params.POINT_3D_OF_ARMOR_BIG, point_2d_of_armor, _cam_instant_matrix, _params.DISTORTION_COEFF, _rVec, _tVec, false,SOLVEPNP_IPPE);
		if(enemy_type == 0)
			solvePnP(_params.POINT_3D_OF_ARMOR_SMALL, point_2d_of_armor, _cam_instant_matrix, _params.DISTORTION_COEFF, _rVec, _tVec, false,SOLVEPNP_IPPE);
		_tVec.at<double>(1, 0) -= _params.Y_DISTANCE_BETWEEN_GUN_AND_CAM;
		xErr = atan(_tVec.at<double>(0, 0) / _tVec.at<double>(2, 0)) / 2 / CV_PI * 360;
		yErr = atan(_tVec.at<double>(1, 0) / _tVec.at<double>(2, 0)) / 2 / CV_PI * 360;
		euclideanDistance = hypot(_tVec.at<double>(0,0),hypot(_tVec.at<double>(1,0),_tVec.at<double>(2,0)))
		if(euclideanDistance >= 8500)
		{
			return TOO_FAR;
		}
		
		return ANGLES_AND_DISTANCE;
	}
	//单点解算
	/*if(angle_solver_algorithm == 0)
	{
		double x1, x2, y1, y2, r2, k1, k2, p1, p2, y_ture;
		x1 = (centerPoint.x - _cam_instant_matrix.at<double>(0, 2)) / _cam_instant_matrix.at<double>(0, 0);
		y1 = (centerPoint.y - _cam_instant_matrix.at<double>(1, 2)) / _cam_instant_matrix.at<double>(1, 1);
		r2 = x1 * x1 + y1 * y1;
		k1 = _params.DISTORTION_COEFF.at<double>(0, 0);
		k2 = _params.DISTORTION_COEFF.at<double>(1, 0);
		p1 = _params.DISTORTION_COEFF.at<double>(2, 0);
		p2 = _params.DISTORTION_COEFF.at<double>(3, 0);
		x2 = x1 * (1 + k1 * r2 + k2 * r2*r2) + 2 * p1*x1*y1 + p2 * (r2 + 2 * x1*x1);
		y2 = y1 * (1 + k1 * r2 + k2 * r2*r2) + 2 * p2*x1*y1 + p1 * (r2 + 2 * y1*y1);
		y_ture = y2 - _params.Y_DISTANCE_BETWEEN_GUN_AND_CAM / 1000;
		xErr = atan(x2) / 2 / CV_PI * 360;
		yErr = atan(y_ture) / 2 / CV_PI * 360;
		
		return ONLY_ANGLES;
	}*/
	
	return ANGLE_ERROR;
}

void AngleSolver::compensateOffset()
{
	//需要了解这个偏移补偿的z是到哪个平面的距离
	/* z of the camera COMS */
	const auto offset_z = 115.0;
	const auto& d = euclideanDistance;
	const auto theta_y = xErr / 180 * CV_PI;
	const auto theta_p = yErr / 180 * CV_PI;
	const auto theta_y_prime = atan((d*sin(theta_y)) / (d*cos(theta_y) + offset_z));
	const auto theta_p_prime = atan((d*sin(theta_p)) / (d*cos(theta_p) + offset_z));
	const auto d_prime = sqrt(pow(offset_z + d * cos(theta_y), 2) + pow(d*sin(theta_y), 2));
	xErr = theta_y_prime / CV_PI * 180;
	yErr = theta_p_prime / CV_PI * 180;
	euclideanDistance = d_prime;
}

void AngleSolver::compensateGravity()
{
	//可能可以改进
	const auto& theta_p_prime = yErr / 180 * CV_PI;
	const auto& d_prime = euclideanDistance;
	const auto& v = bullet_speed;
	const auto theta_p_prime2 = atan((sin(theta_p_prime) - 0.5*9.8*d_prime / pow(v, 2)) / cos(theta_p_prime));
	yErr = theta_p_prime2 / CV_PI * 180;
}


void AngleSolver::set_UserType(int usertype)
{
	user_type = usertype;
}

void AngleSolver::set_EnemyType(int enemytype)
{
	enemy_type = enemytype;
}


void AngleSolver::set_BulletSpeed(int bulletSpeed)
{
	bullet_speed = bulletSpeed;
}

const cv::Vec2f AngleSolver::get_Angle()
{
	return cv::Vec2f(xErr, yErr);
}


double AngleSolver::get_Distance()
{
	return euclideanDistance;
}


//读取摄像头的内参
void AngleSolverParam::readFile(const int id)
{
	cv::FileStorage fsread("../Pose/angle_solver_params.xml", cv::FileStorage::READ);
	if(!fsread.isOpened())
	{
		std::cerr << "failed to open xml" << std::endl;
		return;
	}
	fsread["Y_DISTANCE_BETWEEN_GUN_AND_CAM"] >> Y_DISTANCE_BETWEEN_GUN_AND_CAM;

	switch(id)
	{
	case 0:
	{
		fsread["CAMERA_MARTRIX_0"] >> CAM_MATRIX;
		fsread["DISTORTION_COEFF_0"] >> DISTORTION_COEFF;
		return;
	}
	case 1:
	{
		fsread["CAMERA_MARTRIX_1"] >> CAM_MATRIX;
		fsread["DISTORTION_COEFF_1"] >> DISTORTION_COEFF;
		return;
	}
	case 2:
	{
		fsread["CAMERA_MARTRIX_2"] >> CAM_MATRIX;
		fsread["DISTORTION_COEFF_2"] >> DISTORTION_COEFF;
		return;
	}
	case 3:
	{
		fsread["CAMERA_MARTRIX_3"] >> CAM_MATRIX;
		fsread["DISTORTION_COEFF_3"] >> DISTORTION_COEFF;
		return;
	}
	case 4:
	{
		fsread["CAMERA_MARTRIX_4"] >> CAM_MATRIX;
		fsread["DISTORTION_COEFF_4"] >> DISTORTION_COEFF;
		return;
	}
	case 5:
	{
		fsread["CAMERA_MARTRIX_OFFICIAL"] >> CAM_MATRIX;
		fsread["DISTORTION_COEFF_OFFICIAL"] >> DISTORTION_COEFF;
		return;
	}
	case 6:
	{
		fsread["CAMERA_MARTRIX_6"] >> CAM_MATRIX;
		fsread["DISTORTION_COEFF_6"] >> DISTORTION_COEFF;
		return;
	}
	case 7:
	{
		fsread["CAMERA_MARTRIX_7"] >> CAM_MATRIX;
		fsread["DISTORTION_COEFF_7"] >> DISTORTION_COEFF;
		return;
	}
	case 8:
	{
		fsread["CAMERA_MARTRIX_8"] >> CAM_MATRIX;
		fsread["DISTORTION_COEFF_8"] >> DISTORTION_COEFF;
		return;
	}
	case 9:
	{
		fsread["CAMERA_MARTRIX_9"] >> CAM_MATRIX;
		fsread["DISTORTION_COEFF_9"] >> DISTORTION_COEFF;
		return;
	}
	default:
		std::cout << "wrong cam number given." << std::endl;
		return;
	}
	
}
}