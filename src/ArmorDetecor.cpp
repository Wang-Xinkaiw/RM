#include"ArmorDetecor.hpp"
#include"AngleSolver.hpp"
#include<iostream>
#include<vector>
#include<math.h>
#include<string>
#include<time.h>
using namespace cv;
using namespace cv::ml;
using namespace std;
using namespace rm;
//调试模块
#define TEST_ARMOR_DETECOR

namespace rm
{
enum
{
	WIDTH_GREATER_THAN_HEIGHT,
	ANGLE_TO_UP
};


#ifdef TEST_ARMOR_DETECOR
void showLights(Mat & image, const vector<LightDescription> & lights)
{
    Mat lightDisplay;	//显示灯条用的图像
    image.copyTo(lightDisplay);	//获取源图像的拷贝
    //如果找到了灯条
    if (!lights.empty())
    {
        putText(lightDisplay, "LIGHTS FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 1, 8, false); //title LIGHT_FOUND 大标题 “找到了灯条”
        for (auto& light : lights)
        {
            // Point2f light.points[4];
            // light.points = light.points;
            //draw all the lights' contours 画出所有灯条的轮廓
            for (size_t i = 0; i < 4; i++)
            {
                line(lightDisplay, light.points[i], light.points[(i + 1) % 4], Scalar(255, 0, 255), 1, 8, 0);
            }

            //draw the lights's center point 画出灯条中心
            circle(lightDisplay, light.center, 2, Scalar(0, 255, 0), 2, 8, 0);

            //show the lights' center point x,y value 显示灯条的中心坐标点
            putText(lightDisplay, to_string(int(light.center.x)), light.center, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
            putText(lightDisplay, to_string(int(light.center.y)), light.center + Point2f(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
        }
    }
    //if detector does not find lights 如果没找到灯条
    else
    {
        putText(lightDisplay, "LIGHTS NOT FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 1, 8, false);//title LIGHT_NOT_FOUND 大标题 “没找到灯条”
    }
    //show the result image 显示结果图
    imshow("Lights Monitor", lightDisplay);
	waitKey(1);
    }

/**
*@brief: show all the armors matched in a copy of srcImg  在图像中显示找到的所有装甲板
*/
void showArmors(Mat & image, const vector<ArmorDescription> & armors, const ArmorDescription & targetArmor)
    {
	Mat armorDisplay; //Image for the use of displaying armors 展示装甲板的图像
	image.copyTo(armorDisplay); //get a copy of srcImg 源图像的拷贝 
	// if armors is not a empty vector (ARMOR_FOUND) 如果找到了装甲板
	if (!armors.empty())
	{
		putText(armorDisplay, "ARMOR FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0), 1, 8, false); //title FOUND 大标题 “找到了装甲板”
		//draw all the armors' vertices and center 画出所有装甲板的顶点边和中心
		for (auto armor : armors)
		{
			//draw the center 画中心
			circle(armorDisplay, armor.center, 2, Scalar(0, 255, 0), 2);
			for (size_t i = 0; i < 4; i++)
			{
				line(armorDisplay, armor.vertex[i], armor.vertex[(i + 1) % 4], Scalar(255, 255, 0), 2, 8, 0);
			}
			//display its center point x,y value 显示中点坐标
			putText(armorDisplay, to_string(int(armor.center.x)), armor.center, FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 1, 8, false);
			putText(armorDisplay, to_string(int(armor.center.y)), armor.center + Point2f(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 1, 8, false);
			//putText(armorDisplay, to_string(int(armor.armorNum)), armor.center + Point2f(15, 30), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1, 8, false);
		}
		//connect all the vertices to be the armor contour 画出装甲板轮廓
		for (size_t i = 0; i < 4; i++)
        {
            line(armorDisplay, targetArmor.vertex[i], targetArmor.vertex[(i + 1) % 4], Scalar(255, 255, 255), 2, 8, 0);
        }
    }
    //if armors is a empty vector (ARMOR_NOT FOUND) 如果没找到装甲板
    else
    {
        putText(armorDisplay, "ARMOR NOT FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);//title NOT FOUND 大标题 “没找到装甲板”
    }
    //show the result armors image 显示结果图
    imshow("Armor Monitor", armorDisplay);
	waitKey(1);
}
#endif

cv::RotatedRect& adjustRec(cv::RotatedRect& rec, const int mode)	//调整矩阵
{
	using std::swap;

	float& width = rec.size.width;
	float& height = rec.size.height;
	float& angle = rec.angle;

	if(mode == WIDTH_GREATER_THAN_HEIGHT)	//模式：宽比高大
	{
		if(width < height)	//如果相反，则对调
		{
			swap(width, height);
			angle += 90.0;
		}
	}

	while(angle >= 90.0) angle -= 180.0;
	while(angle < -90.0) angle += 180.0;

	if(mode == ANGLE_TO_UP)	//模式：角度
	{
		if(angle >= 45.0)
		{
			swap(width, height);
			angle -= 90.0;
		}
		else if(angle < -45.0)
		{
			swap(width, height);
			angle += 90.0;
		}
	}

	return rec;
}

ArmorDescription::ArmorDescription()
{
	//numScore = 0;
	vertex.resize(4);
	for(int i = 0; i < 4; i++)
	{
		vertex[i] = cv::Point2f(0, 0);
	}
	type = UNKNOWN_ARMOR;
}

ArmorDescription::ArmorDescription(const LightDescription & l_Light, const LightDescription & r_Light, const int armorType, const cv::Mat & grayImg, ArmorParam _param)
{
	lightPairs[0] = l_Light.rec();
	lightPairs[1] = r_Light.rec();

	cv::Size exLSize(int(lightPairs[0].size.width), int(lightPairs[0].size.height * 2));
	cv::Size exRSize(int(lightPairs[1].size.width), int(lightPairs[1].size.height * 2));
	cv::RotatedRect exLLight(lightPairs[0].center, exLSize, lightPairs[0].angle);
	cv::RotatedRect exRLight(lightPairs[1].center, exRSize, lightPairs[1].angle);

	//set four vertexpoint
	cv::Point2f pts_l[4];
	exLLight.points(pts_l);
	cv::Point2f upper_l = pts_l[2];
	cv::Point2f lower_l = pts_l[3];

	cv::Point2f pts_r[4];
	exRLight.points(pts_r);
	cv::Point2f upper_r = pts_r[1];
	cv::Point2f lower_r = pts_r[0];

	vertex.resize(4);
	vertex[0] = upper_l;
	vertex[1] = upper_r;
	vertex[2] = lower_r;
	vertex[3] = lower_l;
	cv::Line(pst_r[2],)
	center = ((pts_l[2] + pts_l[3])/2 +(pts_r[0] + pts_r[1])/2)/2;

	type = armorType;

	// calculate the distance score
	Point2f srcImgCenter(grayImg.cols / 2, grayImg.rows / 2);
}

ArmorDetector::ArmorDetector(const ArmorParam& armorParam)
{
	param = ArmorParam;
	flag = ARMOR_NO;
}

ArmorDetector::ArmorDetector(const ArmorDetector *armorDetector)
{
	param = armorDetector->armorParam;
	flag = ARMOR_NO;
}


void ArmorDetector::setEnemyColor(ColorChannels enemy_color)
{
	enemy_color = enemy_color;
	self_color = enemy_color == BLUE ? RED : BLUE;
}

int ArmorDetector::detect()
{
	armors.clear();

	AngleSolverParam angleParam;

	std::vector<LightDescription> lightInfos;
	{	
		vector<Mat> channels;
		//视频流
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

		/*
		split(srcImg,channels);
		if (!channels.empty() && channels.size() >= 3) 
		{
			if(enemy_color == RED)
    			{
					grayImg = channels.at(2)-channels.at(0);
				}
 			else grayImg = channels.at(0)-channels.at(2);
		}
		else cout<<"ERROR at channels"<<endl;
		*/
		cvtColor(srcImg, grayImg, COLOR_BGR2GRAY);  
		
		cv::Mat beBright_Img;

		//grayImg = GaussianBlur(grayImg, (5, 5), 0);
		
		cv::threshold(grayImg, beBright_Img, param.brightness_threshold, 255, cv::THRESH_BINARY);

		cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, Size(3,3));
		dilate(beBright_Img, beBright_Img, element);

		//imshow("dilate img",beBright_Img);
		//waitKey(1);
		
		std::vector<vector<Point>> lightContours;
		cv::findContours(beBright_Img.clone(), lightContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		for(const auto& contour : lightContours)
		{
			float lightContourArea = contourArea(contour);
			if(contour.size() <= 5 || lightContourArea < param.light_min_area) continue;

			RotatedRect lightRec = fitEllipse(contour);

			adjustRec(lightRec, ANGLE_TO_UP);

			if(lightRec.size.width / lightRec.size.height > param.light_max_ratio ||
			   lightContourArea / lightRec.size.area() < param.light_contour_min_solidity) continue;

			lightRec.size.width *= param.light_color_detect_extend_ratio;
			lightRec.size.height *= param.light_color_detect_extend_ratio;
		
			cv::Rect lightRect = lightRec.boundingRect();
			
			lightInfos.push_back(LightDescription(lightRec));

			//Point2f vertice[4];
			//lightRec.points(vertice);
			//for (int i = 0; i < 4; i++){
			//line(beBright_Img, vertice[i], vertice[(i + 1) % 4], Scalar(255, 0, 0), 2);
			//在图像 FirstResult 上绘制线段，连接旋转矩形的相邻顶点。线段的颜色为青色（绿色和蓝色的混合，由 Scalar(0, 255, 255) 指定），线宽为2个像素。(i + 1) % 4 确保在绘制最后一个顶点后，线条会连接回第一个顶点，形成一个闭合的四边形
			//cv::putText(beBright_Img, std::to_string(lightRec.angle), lightRec.center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);}
			//绘制旋转矩形的角度值
			//imshow("FirstResult",beBright_Img);
			//waitKey(0);
			

		}
		if(lightInfos.empty())
		{	
			cout<<"ERROR at lightInfos"<<endl;
			return flag = ARMOR_NO;
		}

		{	
		sort(lightInfos.begin(), lightInfos.end(), [](const LightDescription& ld1, const LightDescription& ld2)
		{
			return ld1.center.x < ld2.center.x;
		});
		
		for(size_t i = 0; i < lightInfos.size(); i++)
		{
			for(size_t j = i + 1; (j < lightInfos.size()); j++)
			{
				const LightDescription& leftLight=lightInfos[i];
				const LightDescription& rightLight=lightInfos[j];
			
				float angleDiff_ = abs(leftLight.angle - rightLight.angle);
			
				float LenDiff_ratio = abs(leftLight.length - rightLight.length) / max(leftLight.length, rightLight.length);
				
				if(angleDiff_ > param.light_max_angle_diff_ ||
				   LenDiff_ratio > param.light_max_height_diff_ratio_)
				{
					continue;
				}

				float dis = sqrt((leftLight.center.x-rightLight.center.x)*(leftLight.center.x-rightLight.center.x)+(leftLight.center.y-rightLight.center.y)*(leftLight.center.y-rightLight.center.y));

				float meanLen = (leftLight.length + rightLight.length) / 2;
				
				float yDiff = abs(leftLight.center.y - rightLight.center.y);
			
				float yDiff_ratio = yDiff / meanLen;
			
				float xDiff = abs(leftLight.center.x - rightLight.center.x);
			
				float xDiff_ratio = xDiff / meanLen;
			
				float ratio = dis / meanLen;
		
				if(yDiff_ratio > param.light_max_y_diff_ratio_ ||
				   xDiff_ratio < param.light_min_x_diff_ratio_ ||
				   ratio > param.armor_max_aspect_ratio_ ||
				   ratio < param.armor_min_aspect_ratio_)
				{
					continue;
				}

				
				int armorType = ratio > param.armor_big_armor_ratio ? BIG_ARMOR : SMALL_ARMOR;

				/*
				计算旋转得分
				float ratiOff = (armorType == BIG_ARMOR) ? max(param.armor_big_armor_ratio - ratio, float(0)) : max(param.armor_small_armor_ratio - ratio, float(0));
				float yOff = yDiff / meanLen;
				float rotationScore = -(ratiOff * ratiOff + yOff * yOff);
				*/
				
				ArmorDescription armor(leftLight, rightLight, armorType, grayImg, param);
				armors.emplace_back(armor);
				break;
			}
			
		}
	
		//if(armors.empty())
		//{
		//	return flag = ARMOR_NO;
		//}

			/*
		armors.erase(remove_if(armors.begin(), armors.end(), [](ArmorDescription& i)
		{
			return !(i.isArmorPattern());
		}), armors.end());
		//如果全都不是装甲板
		if(armors.empty())
		{	//清空目标装甲板
			targetArmor.clear();

			if(flag == ARMOR_LOCAL)
			{
				return flag = ARMOR_LOST;
			}
			else
			{
				return flag = ARMOR_NO;
			}
		}
		for(auto & armor : armors)
		{
			armor.finalScore = armor.sizeScore + armor.distScore + armor.rotationScore;
		}

		//choose the one with highest score, store it on _targetArmor
		std::sort(armors.begin(), armors.end(), [](const ArmorDescription & a, const ArmorDescription & b)
		{
			return a.finalScore > b.finalScore;
		});
		targetArmor = armors[0];

		//update the flag status	
		trackCnt++;

		return flag = ARMOR_LOCAL;
		*/

		}

		angleParam.readFile(1);//only 0 1 2 
		AngleSolver angleSolver;
		AngleSolver();
		angleSolver.init(angleParam);
		
		for(auto& armor :armors)
		{	
			angleSolver.set_EnemyType(1);
			angleSolver.set_Target(armor.vertex, 1);
			cout<<armor.vertex<<endl;
			angleSolver.solve();
		}
		
		cout<<angleSolver.get_Angle()<<endl;
		cout<<angleSolver.get_Distance()<<endl;
		waitKey(10);
		showLights(srcImg, lightInfos);
		showArmors(srcImg, armors, targetArmor);
		if(!lightInfos.empty())
		{
			lightInfos.clear();
			
		}
		
		if(!armors.empty())
		{
			armors.clear();
			targetArmor.clear();
		}
		}
	}
	srcImg.release();
	destroyAllWindows();
}

// int getArmorType()
// {
// 	return ArmorDescription::type;
// }

}



