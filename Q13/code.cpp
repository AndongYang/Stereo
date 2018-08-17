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

const string g_read_photo_list_path_l = "left_photo_list.xml";		//左相机拍摄的图片列表
const string g_read_photo_list_path_r = "right_photo_list.xml";	//右相机拍摄的图像列表
const string g_read_path = "stereoCalibrate_res.xml";			//存放了stereoCalibrate函数输出的左右相机矩阵，畸变参数，转换矩阵R，T

//获取stereoCalibrate函数输出的转换矩阵与各个相机参数
bool get_camera_matrix_and_distCoeffs(Mat &camera_matrix_l, Mat &distortion_coeffs_l,
	Mat &camera_matrix_r, Mat &distortion_coeffs_r, Mat &R, Mat &T) {
	cout << "Start get_camera_matrix_and_distCoeffs" << endl;
	FileStorage fs(g_read_path, FileStorage::READ);
	if (!fs.isOpened())
	{
		cerr << "Xml failed to open." << endl;
		return false;
	}

	fs["camera_matrix_l"] >> camera_matrix_l;
	fs["distortion_coeffs_l"] >> distortion_coeffs_l;
	fs["camera_matrix_r"] >> camera_matrix_r;
	fs["distortion_coeffs_r"] >> distortion_coeffs_r;
	fs["R"] >> R;
	fs["T"] >> T;

	fs.release();
	cout << "Get_camera_matrix_and_distCoeffs completed." << endl;
	return true;
}

//获得要校正的左右相机图像路径
vector<string> get_file_list(string photo_list_path) {
	cout << "Strat read xml." << endl;
	vector<string> file_list;
	const string file_name = photo_list_path;

	FileStorage fs(file_name, FileStorage::READ);
	if (!fs.isOpened())
	{
		cerr << "Xml failed to open." << endl;
	}

	FileNode n = fs["images"];
	if (n.type() != FileNode::SEQ)
	{
		cerr << "Images is not a sequence! FAIL." << endl;
	}
	cout << "Get file_list:" << endl;
	for (FileNodeIterator it = n.begin(); it != n.end(); it++)
	{
		file_list.push_back((string)(*it));
		cout << (string)(*it) << endl;
	}
	fs.release();
	cout << "Get file_list:" << photo_list_path << " completed." << endl;
	return file_list;
}

void go_rectify(vector<string> &file_list_l, vector<string> &file_list_r, 
	Mat &camera_matrix_l, Mat &distortion_coeffs_l,Mat &camera_matrix_r,
	Mat &distortion_coeffs_r, Size &image_size, Mat &R, Mat &T) {
	cout << "Start go_rectify." << endl;
	int image_counter = 0;
	Rect validroi_l, validroi_r;
	Mat R_l, R_r, P_l, P_r, Q;
	Mat map_l_1, map_l_2, map_r_1, map_r_2;
	Mat img_l, img_r, rectify_img_l, rectify_img_r;

	//为每个摄像头计算立体校正的映射矩阵。
	//其运行结果并不是直接将图片进行立体矫正，而是得出进行立体矫正所需要的映射矩阵。
	stereoRectify(camera_matrix_l, distortion_coeffs_l, camera_matrix_r, distortion_coeffs_r,
		image_size, R, T, R_l, R_r, P_l, P_r, Q, CALIB_ZERO_DISPARITY, 1, Size(0,0), &validroi_l, &validroi_r);

	//根据映射矩阵分别计算左右相机的畸变矫正和立体校正的映射变换
	initUndistortRectifyMap(camera_matrix_l, distortion_coeffs_l, R_l, P_l, image_size, 
		CV_16SC2, map_l_1, map_l_2);
	initUndistortRectifyMap(camera_matrix_r, distortion_coeffs_r, R_r, P_r, image_size, 
		CV_16SC2, map_r_1, map_r_2);

	int flag = 0;

	for (image_counter = 0; image_counter < file_list_l.size(); image_counter++) {
		img_l = imread(file_list_l[image_counter], 1);
		img_r = imread(file_list_r[image_counter], 1);
	
		//调用几何变换函数，将左右相机像平面对齐
		remap(img_l, rectify_img_l, map_l_1, map_l_2, INTER_LINEAR);
		remap(img_r, rectify_img_r, map_r_1, map_r_2, INTER_LINEAR);
	
		//将校正前后图像输出
		flag += 1;
		Mat canvas(image_size.height, image_size.width * 2, CV_8UC3);
		Mat can_left = canvas(Rect(0, 0, image_size.width, image_size.height));
		Mat can_right = canvas(Rect(image_size.width, 0, image_size.width, image_size.height));
		img_l.copyTo(can_left);
		img_r.copyTo(can_right);
		imshow("Before Rectify", canvas);
		waitKey(500);

		rectify_img_l.copyTo(can_left);
		rectify_img_r.copyTo(can_right);

		/*画上对应的线条*/
		for (int i = 0; i < canvas.rows; i += 16)
			line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);

		imshow("Rectified", canvas);
		waitKey(500);
	}
	cout << "Start go_rectify completed." << endl;
}
int main() {
	Size image_size = Size(640, 480);

	//获取左右图像文件路径列表
	vector<string> file_list_l, file_list_r;
	file_list_l = get_file_list(g_read_photo_list_path_l);
	file_list_r = get_file_list(g_read_photo_list_path_r);

	//获取stereoCalibrate()输出的各个参数
	Mat camera_matrix_l, distortion_coeffs_l, camera_matrix_r, distortion_coeffs_r, R, T;
	get_camera_matrix_and_distCoeffs(camera_matrix_l, distortion_coeffs_l,
		camera_matrix_r, distortion_coeffs_r, R, T);

	//开始进行立体校正
	go_rectify(file_list_l, file_list_r, camera_matrix_l, distortion_coeffs_l,
		camera_matrix_r, distortion_coeffs_r, image_size, R, T);

	system("pause");
	return 0;
}
