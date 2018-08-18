#include <iostream> 
#include <string>
#include <fstream>
#include <io.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc.hpp> 

using namespace cv;
using namespace std;

/*Created by yangandong*/

const string g_read_xml_path = "left_photo_list.xml";
const string g_save_xml_path = "camera_matrix_l.xml";

//获取待标定相机拍摄的一组图片
vector<string> get_file_list() {
	vector<string> file_list;
	const string file_name = g_read_xml_path;

	FileStorage fs(file_name, FileStorage::READ);
	if (!fs.isOpened())
	{
		cerr << "xml failed to open "<< endl;
	}

	FileNode n = fs["images"];
	if (n.type() != FileNode::SEQ)
	{
		cerr << "images is not a sequence! FAIL" << endl;
	}
	cout << "get file_list:" << endl;
	for (FileNodeIterator it = n.begin(); it != n.end(); it++)
	{
		file_list.push_back((string)(*it));
		cout << (string)(*it) << endl;
	}
	fs.release();
	cout << "get file_list success." << endl;
	return file_list;
}

//此函数包含了棋盘角点寻找与相机标定两个过程
void cal_calibratecamera(vector<string> &file_list, Mat &cameraMatrix,
	Mat &distCoeffs, vector<Mat> &rvecsMat, vector<Mat> &tvecsMat){
	cout << "Find chessboard corners" << endl;
	int image_counter = 0;
	Size square_size = Size(1.0, 1.0); //棋盘格中每个小方块的大小 
	Size image_size;
	Size board_corner_number = Size(9, 6);
	vector<vector<Point2f>> all_image_corners;
	vector<vector<Point3f>> all_object_points;

	for (image_counter = 0; image_counter < file_list.size(); image_counter++) {
		Mat img = imread(file_list[image_counter]);
		Mat img_gray;
		image_size.width = img.cols;
		image_size.height = img.rows;
		cvtColor(img, img_gray, COLOR_BGR2GRAY);
		//调用函数，寻找棋盘角点
		vector<Point2f> one_image_corners;
		bool found = findChessboardCorners(img, board_corner_number, one_image_corners,
			CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

		if (found == 0) {	//如果寻找角点失败
			cout << "findChessboardCorners() failed, image path:" << file_list[image_counter] << endl;
		}
		else {	//如果寻找角点成功
			//提高角点坐标精确度
			cornerSubPix(img_gray, one_image_corners, Size(11, 11), Size(-1, -1),
				TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));

			all_image_corners.push_back(one_image_corners);

			//为棋盘角点创建3D坐标
			vector<Point3f> object_points;
			for (int i = 0; i < board_corner_number.height; i++) {
				for (int j = 0; j < board_corner_number.width; j++) {
					Point3f coordinate_3D;
					//Assuming z =0;
					coordinate_3D.x = i * square_size.height;
					coordinate_3D.y = j * square_size.width;
					coordinate_3D.z = 0;
					object_points.push_back(coordinate_3D);
				}
			}
			all_object_points.push_back(object_points);

			//在图像上画出角点
			//drawChessboardCorners(img, board_corner_number, one_image_corners, found);

			//显示图片
			//imshow(file_list[image_counter], img);
			//waitKey(150);
		}
	}
	cout << "Find chessboard corners success." << endl;

	cout << "calibrateCamera" << endl;
	//进行相机标定
	double co = calibrateCamera(all_object_points, all_image_corners, image_size,
		cameraMatrix, distCoeffs,
		rvecsMat, tvecsMat, CV_CALIB_FIX_K4);
	cout << "calibrateCamera success." << endl;

	//保存标定结果
	cout << "save camera matrix and distCoeffs matrix" << endl;
	FileStorage fs(g_save_xml_path, FileStorage::WRITE);
	if (!fs.isOpened())
	{
		cerr << "failed to open " << endl;
	}
	fs << "cameraMatrix" << cameraMatrix;
	fs << "distCoeffs" << distCoeffs;
	fs.release();
	cout << "save camera matrix and distCoeffs matrix success." << endl;
}

int main()
{  
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));        	// 相机内参矩阵
	Mat distCoeffs = Mat(1, 14, CV_32FC1, Scalar::all(0));          // 畸变参数[k1,k2,k3,p1,p2]
	vector<Mat> rvecsMat;                                          	// 旋转向量 R
	vector<Mat> tvecsMat;						// 位移向量 T
	
	vector<string> file_list = get_file_list();
	cal_calibratecamera(file_list, cameraMatrix, distCoeffs, rvecsMat, tvecsMat);

	system("pause");
	return 0;
}
