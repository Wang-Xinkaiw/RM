#include<iostream>
#include"AngleSolver_test.hpp"
#include "opencv2/opencv.hpp"
#include "math.h"

using namespace cv;
using namespace std;
using namespace rm;

namespace rm
{
std::vector<cv::Point3f> AngleSolverParam::POINT_3D_OF_ARMOR_BIG = std::vector<cv::Point3f>
{

		cv::Point3f(-105, -30, 0),	//tl
		cv::Point3f(105, -30, 0),	//tr
		cv::Point3f(105, 30, 0),	//br
		cv::Point3f(-105, 30, 0)	//bl
};
std::vector<cv::Point3f> AngleSolverParam::POINT_3D_OF_RUNE = std::vector<cv::Point3f>
{
	cv::Point3f(-370, -220, 0),
	cv::Point3f(0, -220, 0),
	cv::Point3f(370, -220, 0),
	cv::Point3f(-370, 0, 0),
	cv::Point3f(0, 0, 0),
	cv::Point3f(370, 0, 0),
	cv::Point3f(-370, 220, 0),
	cv::Point3f(0, 220, 0),
	cv::Point3f(370, 220, 0)
};

std::vector<cv::Point3f> AngleSolverParam::POINT_3D_OF_ARMOR_SMALL = std::vector<cv::Point3f>
{
	cv::Point3f(-60, -60, 0),	//tl
	cv::Point3f(60, -60, 0),	//tr
	cv::Point3f(60, 60, 0),		//br
	cv::Point3f(-60, 60, 0)		//bl
};

AngleSolver::AngleSolver()
{
	for(int ll = 0; ll <= 3; ll++)
		target_nothing.push_back(cv::Point2f(0.0, 0.0));
};

void AngleSolver::init(const AngleSolverParam& AngleSolverParam)
{
	param = AngleSolverParam;
	_cam_instant_matrix = param.CAM_MATRIX.clone();
};

//algorithm=0 pnp 1 pinhole
void AngleSolver::set_Target(const std::vector<cv::Point2f> objectPoints, int objectType)//objectType=0 BIG 1 SMALL
{
	if(objectType == 0 || objectType == 1)
	{
		if(angle_solver_algorithm == 1 || angle_solver_algorithm == 2)
		{
			angle_solver_algorithm = 0; cout << "algorithm is reset to PNP Solution" << endl;
		}
		point_2d_of_armor = objectPoints;
		if(objectType == 0)
			enemy_type = 0;
		else
			enemy_type = 1;
		return;
	}
	// if(objectType == 3 || objectType == 4)
	// {
	// 	angle_solver_algorithm = 2;
	// 	point_2d_of_rune = objectPoints;
	// }

}

void AngleSolver::set_Target(const cv::Point2f Center_of_armor, int objectPoint)
{
	if(angle_solver_algorithm == 0 || angle_solver_algorithm == 2)
	{
		angle_solver_algorithm = 1; cout << "algorithm is reset to One Point Solution" << endl;
	}
	centerPoint = Center_of_armor;
	// if(objectPoint == 3 || objectPoint == 4)
	// 	is_shooting_rune = 1;
	// else
	// {
	// 	is_shooting_rune = 0;
	// 	rune_compensated_angle = 0;
	// }
}

#ifdef DEBUG

void AngleSolver::showPoints2dOfArmor()
{
	cout << "the point 2D of armor is" << point_2d_of_armor << endl;
}


void AngleSolver::showTvec()
{
	cv::Mat tvect;
	transpose(t_Vec, tvect);
	cout << "the current t_Vec is:" << endl << tvect << endl;
}

void AngleSolver::showEDistance()
{
	cout << "  _euclideanDistance is  " << euclideanDistance / 1000 << "m" << endl;
}

void AngleSolver::showcenter_of_armor()
{
	cout << "the center of armor is" << centerPoint << endl;
}

void AngleSolver::showAngle()
{
	cout << "_xErr is  " << xErr << "  _yErr is  " << yErr << endl;
}

int AngleSolver::showAlgorithm()
{
	return angle_solver_algorithm;
}
#endif // DEBUG

AngleSolver::AngleFlag AngleSolver::solve()
{
	if(angle_solver_algorithm == 0)
	{
		if(enemy_type == 0)
			solvePnP(param.POINT_3D_OF_ARMOR_BIG, point_2d_of_armor, _cam_instant_matrix, param.DISTORTION_COEFF, r_Vec, t_Vec, false,SOLVEPNP_IPPE);
		if(enemy_type == 1)
			solvePnP(param.POINT_3D_OF_ARMOR_SMALL, point_2d_of_armor, _cam_instant_matrix, param.DISTORTION_COEFF, r_Vec, t_Vec, false,SOLVEPNP_IPPE);
		t_Vec.at<double>(1, 0) -= param.Y_DISTANCE_BETWEEN_GUN_AND_CAM;
		euclideanDistance = sqrt(t_Vec.at<double>(0, 0)*t_Vec.at<double>(0, 0) + t_Vec.at<double>(1, 0)*t_Vec.at<double>(1, 0) + t_Vec.at<double>(2, 0)* t_Vec.at<double>(2, 0));
		if(euclideanDistance >= 5000)
		{
		double fx = _cam_instant_matrix.at<double>(0,0);
		double fy = _cam_instant_matrix.at<double>(1,1);
		double cx = _cam_instant_matrix.at<double>(0,2);
		double cy = _cam_instant_matrix.at<double>(1,2);
		cv::Point2f pnt;
		vector<cv::Point2f> in;
		vector<cv::Point2f> out;
		in.push_back(Point2f(centerPoint));
	
		undistortPoints(in,out,_cam_instant_matrix,param.DISTORTION_COEFF,noArray(),_cam_instant_matrix);
		pnt=out.front();

		double rx=(pnt.x-cx)/fx;
		double ry=(pnt.y-cy)/fy;
		double ry_new = ry - param.Y_DISTANCE_BETWEEN_GUN_AND_CAM / 1000;

		xErr = atan(rx) / CV_PI * 180;
		yErr = -atan(ry_new) / CV_PI * 180;
		}
		else
		{
		xErr = atan(t_Vec.at<double>(0, 0) / t_Vec.at<double>(2, 0)) / CV_PI * 180;
		yErr = -atan(t_Vec.at<double>(1, 0) / sqrt(t_Vec.at<double>(2, 0)*t_Vec.at<double>(2,0) + t_Vec.at<double>(0,0)*t_Vec.at<double>(0,0)) )/ CV_PI * 180;
		}
	}
	if(angle_solver_algorithm == 1)
	{   
        double fx = _cam_instant_matrix.at<double>(0,0);
        double fy = _cam_instant_matrix.at<double>(1,1);
        double cx = _cam_instant_matrix.at<double>(0,2);
        double cy = _cam_instant_matrix.at<double>(1,2);
        cv::Point2f pnt;
        vector<cv::Point2f> in;
        vector<cv::Point2f> out;
        in.push_back(Point2f(centerPoint));
        
        undistortPoints(in,out,_cam_instant_matrix,param.DISTORTION_COEFF,noArray(),_cam_instant_matrix);
        pnt=out.front();

        double rx=(pnt.x-cx)/fx;
        double ry=(pnt.y-cy)/fy;
        double ry_new = ry - param.Y_DISTANCE_BETWEEN_GUN_AND_CAM / 1000;

        xErr = atan(rx) / CV_PI * 180;
	    yErr = -atan(ry) / CV_PI * 180;

		if(is_shooting_rune) yErr -= rune_compensated_angle;
		
		return ONLY_ANGLE;
	}
	if(angle_solver_algorithm == 2)
	{
		std::vector<Point2f> runeCenters;
		std::vector<Point3f> realCenters;
		for(size_t i = 0; i < 9; i++)
		{
			if(point_2d_of_rune[i].x > 0 && point_2d_of_rune[i].y > 0)
			{
				runeCenters.push_back(point_2d_of_rune[i]);
				realCenters.push_back(param.POINT_3D_OF_RUNE[i]);
			}
		}

		solvePnP(realCenters, runeCenters, _cam_instant_matrix, param.DISTORTION_COEFF, r_Vec, t_Vec, false);
		t_Vec.at<double>(1, 0) -= param.Y_DISTANCE_BETWEEN_GUN_AND_CAM;
		xErr = atan(t_Vec.at<double>(0, 0) / t_Vec.at<double>(2, 0)) / 2 / CV_PI * 360;
		yErr = atan(t_Vec.at<double>(1, 0) / t_Vec.at<double>(2, 0)) / 2 / CV_PI * 360;
		euclideanDistance = sqrt(t_Vec.at<double>(0, 0)*t_Vec.at<double>(0, 0) + t_Vec.at<double>(1, 0)*t_Vec.at<double>(1, 0) + t_Vec.at<double>(2, 0)* t_Vec.at<double>(2, 0));
		if(euclideanDistance >= 8500)
		{
			return TOO_FAR;
		}
		
		return ANGLES_AND_DISTANCE;
	}
	return ANGLE_ERROR;
}

void AngleSolver::compensateOffset()
{
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

void AngleSolver::set_CAM_SIZE(int width,int height)
{
	image_size.width = width;
	image_size.height = height;
}

//const cv::Vec2f AngleSolver::getCompensateAngle()
//{
//	return cv::Vec2f(_xErr, _yErr);
//}

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

//void AngleSolver::selectAlgorithm(const int t)
//{
//	if (t == 0 || t == 1)
//		angle_solver_algorithm = t;
//
//}

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
	case 0://bubing
	{
		fsread["CAMERA_MARTRIX_0"] >> CAM_MATRIX;
		fsread["DISTORTION_COEFF_0"] >> DISTORTION_COEFF;
		return;
	}
	case 1://shaobing
	{
		fsread["CAMERA_MARTRIX_1"] >> CAM_MATRIX;
		fsread["DISTORTION_COEFF_1"] >> DISTORTION_COEFF;
		return;
	}
	case 2://yingxiong
	{
		fsread["CAMERA_MARTRIX_2"] >> CAM_MATRIX;
		fsread["DISTORTION_COEFF_2"] >> DISTORTION_COEFF;
		return;
	}
	default:
		std::cout << "wrong cam number given." << std::endl;
		return;
	}

}
}