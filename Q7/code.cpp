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

const string g_read_camera_matrix_path = "camera_matrix_l.xml";
const string g_read_photo_list_path = "left_photo_list.xml";
const string g_save_path = "./";

//获取相机的相机矩阵与畸变参数
bool get_camera_matrix_and_distCoeffs(Mat &camera_matrix, Mat &distortion_coeffs) {
	FileStorage fs(g_read_camera_matrix_path, FileStorage::READ);
	if (!fs.isOpened())
	{
		cerr << "Xml failed to open " << endl;
		return false;
	}

	fs["cameraMatrix"] >> camera_matrix;
	fs["distCoeffs"] >> distortion_coeffs;

	fs.release();
	return true;
}

//获取待处理的图片列表
vector<string> get_file_list() {
	cout << "Strat read xml" << endl;
	vector<string> file_list;
	const string file_name = g_read_photo_list_path;

	FileStorage fs(file_name, FileStorage::READ);
	if (!fs.isOpened())
	{
		cerr << "Xml failed to open " << endl;
	}

	FileNode n = fs["images"];
	if (n.type() != FileNode::SEQ)
	{
		cerr << "Images is not a sequence! FAIL" << endl;
	}
	cout << "Get file_list:" << endl;
	for (FileNodeIterator it = n.begin(); it != n.end(); it++)
	{
		file_list.push_back((string)(*it));
		cout << (string)(*it) << endl;
	}
	fs.release();
	cout << "Get file_list success." << endl;
	return file_list;
}

//进行畸变矫正
void do_undistort(vector<string> file_list, Mat camera_matrix, Mat distortion_coeffs) {
	cout << "Start undistort" << endl;
	int image_counter = 0;
	//遍历图片列表，依次处理图片
	for (image_counter = 0; image_counter < file_list.size(); image_counter++) {
		//读取一张图片
		Mat one_image = imread(file_list[image_counter]);
		Mat one_image_res = one_image.clone();

		//对图片进行畸变矫正
		undistort(one_image, one_image_res, camera_matrix, distortion_coeffs);

		//存储矫正后的图片
		string write_path = g_save_path + "res_" + to_string(image_counter) + ".jpg";
		imwrite(write_path, one_image_res);
		cout << "Photo" << image_counter << " finished" << endl;
	}
	cout << "Undistort success." << endl;
}

int main()
{
	Mat camera_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));       // 相机内参
	Mat distortion_coeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));   // 畸变参数[k1,k2,k3,p1,p2]
	vector<Mat> rvecsMat;                                          // 旋转矩阵 R
	vector<Mat> tvecsMat;										   // 位移矩阵 T

	//获取待处理图片列表
	vector<string> file_list = get_file_list();						

	if (get_camera_matrix_and_distCoeffs(camera_matrix, distortion_coeffs)) {
		 //如果获取相机相关参数成功，则进行畸变矫正
		cout << "Read XML success." << endl;
		do_undistort(file_list, camera_matrix, distortion_coeffs);
	}
	else //如果获取相机相关参数失败，则输出错误信息
		cout << "Read XML failed." << endl;

	system("pause");
	return 0;
}
