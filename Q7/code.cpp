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

const string g_read_camera_matrix_path = "camera_matrix.xml";
const string g_read_photo_list_path = "left_photo_list.xml";
const string g_save_path = "./res/";

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

void do_undistort(vector<string> file_list, Mat camera_matrix, Mat distortion_coeffs) {
	cout << "Start undistort" << endl;
	int image_counter = 0;
	for (image_counter = 0; image_counter < file_list.size(); image_counter++) {
		Mat one_image = imread(file_list[image_counter]);
		Mat one_image_res = one_image.clone();

		undistort(one_image, one_image_res, camera_matrix, distortion_coeffs);

		string write_path = g_save_path + to_string(image_counter) + ".jpg";
		imwrite(write_path, one_image_res);
		cout << "Photo" << image_counter << " finished" << endl;
	}
	cout << "Undistort success." << endl;
}

int main()
{
	Mat camera_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));       // Intrinsic matrix
	Mat distortion_coeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));   // Distortion factor[k1,k2,k3,p1,p2]
	vector<Mat> rvecsMat;                                          // Extrinsic parameters R
	vector<Mat> tvecsMat;										   // Extrinsic parameters T

	vector<string> file_list = get_file_list();					   // Get images

	if (get_camera_matrix_and_distCoeffs(camera_matrix, distortion_coeffs)) {
		cout << "Read XML success." << endl;
		do_undistort(file_list, camera_matrix, distortion_coeffs);
	}
	else
		cout << "Read XML failed." << endl;

	return 0;
}
