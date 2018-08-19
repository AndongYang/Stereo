#include <iostream> 
#include <string.h>
#include <fstream>
#include <io.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc.hpp> 

using namespace cv;
using namespace std;

/*Created by yangandong*/

const string g_read_photo_path = "rectify_img.xml";				//存放了经过校正的图片对

//获取待处理图像对
bool get_photo(Mat &rectify_img_l, Mat &rectify_img_r) {
	cout << "Start get_photo" << endl;
	FileStorage fs(g_read_photo_path, FileStorage::READ);
	if (!fs.isOpened())
	{
		cerr << "Xml failed to open." << endl;
		return false;
	}

	//读取左右图像
	fs["rectify_img_l"] >> rectify_img_l;
	fs["rectify_img_r"] >> rectify_img_r;

	fs.release();
	cout << "Get_photo completed." << endl;
	return true;
}

void go_disparity_maps(Mat &rectify_img_l, Mat &rectify_img_r) {
	cout << "Start go_disparity_maps" << endl;

	//创建sgbm类指针
	Ptr<StereoSGBM> sgbm = StereoSGBM::create();
	int SADWindowSize = 8;
	//预处理sobel，获得图像梯度信息，用于计算代价
	sgbm->setPreFilterCap(63);
	//代价参数，得到SAD代价
	Size image_size = Size(640, 480);
	int numberOfDisparities = 64;
	int sgbmWinSize = SADWindowSize;
	sgbm->setBlockSize(sgbmWinSize);
	sgbm->setMinDisparity(0);
	sgbm->setNumDisparities(numberOfDisparities);

	//动态规划参数，默认四条路径
	int cn = rectify_img_l.channels();
	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);

	//后处理参数，唯一性检测、亚像素插值、左右一致性检测、连通区域检测
	sgbm->setUniquenessRatio(10);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setSpeckleRange(32);
	sgbm->setDisp12MaxDiff(1);
	
	//设置模式进行深度计算
	Mat res_temp, res;
	sgbm->setMode(StereoSGBM::MODE_SGBM);
	sgbm->compute(rectify_img_l, rectify_img_r, res_temp);
	
	//显示原图片对
	Mat canvas(image_size.height, image_size.width * 2, CV_8UC3);
	Mat can_left = canvas(Rect(0, 0, image_size.width, image_size.height));
	Mat can_right = canvas(Rect(image_size.width, 0, image_size.width, image_size.height));
	rectify_img_l.copyTo(can_left);
	rectify_img_r.copyTo(can_right);
	imshow("Origin photos", canvas);
	imwrite("C:/Users/AnDon/Desktop/temp/Q17_1.png", canvas);
	waitKey(500);

	//显示视差图
	cout << "Show image" << endl;
	//将compute函数输出的结果除以16得到真实的视差值
	res_temp.convertTo(res_temp, CV_32F, 255 / (numberOfDisparities*16.));
	//显示结果图像
	normalize(res_temp, res, 0, 255, NORM_MINMAX, CV_8UC1);
	imshow("disparity maps", res);
	
	Mat res_2;
	res_temp.convertTo(res_2, CV_8U);
	imshow("disparity maps_2", res_2);

	imwrite("C:/Users/AnDon/Desktop/temp/Q17_2.png", res);
	waitKey(0);

	cout << "Go_disparity_maps completed" << endl;
}

int main() {
	Mat rectify_img_l, rectify_img_r;

	//获取要计算视差图的图像对
	get_photo(rectify_img_l, rectify_img_r);

	//计算视差图
	go_disparity_maps(rectify_img_l, rectify_img_r);

	system("pause");
	return 0;
}
