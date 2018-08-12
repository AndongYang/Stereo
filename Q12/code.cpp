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

const string g_read_camera_matrix_path_l = "camera_matrix_l.xml";
const string g_read_camera_matrix_path_r = "camera_matrix_r.xml";
const string g_read_photo_list_path_l = "left_photo_list.xml";
const string g_read_photo_list_path_r = "right_photo_list.xml";
const string g_save_path = "res.xml";

bool get_camera_matrix_and_distCoeffs(Mat &camera_matrix_l, Mat &distortion_coeffs_l, 
	Mat &camera_matrix_r, Mat &distortion_coeffs_r) {
	FileStorage fs_l(g_read_camera_matrix_path_l, FileStorage::READ);
	if (!fs_l.isOpened())
	{
		cerr << "Xml failed to open." << endl;
		return false;
	}

	fs_l["cameraMatrix"] >> camera_matrix_l;
	fs_l["distCoeffs"] >> distortion_coeffs_l;

	fs_l.release();

	FileStorage fs_r(g_read_camera_matrix_path_r, FileStorage::READ);
	if (!fs_r.isOpened())
	{
		cerr << "Xml failed to open." << endl;
		return false;
	}

	fs_r["cameraMatrix"] >> camera_matrix_r;
	fs_r["distCoeffs"] >> distortion_coeffs_r;

	fs_r.release();
	return true;
}

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

void get_points(vector<string> &file_list_l, vector<string> &file_list_r, vector<vector<Point2f>> &all_image_corners_l,
	vector<vector<Point2f>> &all_image_corners_r, vector<vector<Point3f>> &all_object_points, Size &image_size){
	int image_counter = 0;
	Size square_size = Size(20, 20); //The size of a square, in mm 
	Size board_corner_number = Size(9, 6);
	vector<Point2f> one_image_corners;

	for (image_counter = 0; image_counter < file_list_l.size(); image_counter++) {
		Mat img_l = imread(file_list_l[image_counter], CV_LOAD_IMAGE_GRAYSCALE);
		Mat img_r = imread(file_list_r[image_counter], CV_LOAD_IMAGE_GRAYSCALE);

		bool found_l = findChessboardCorners(img_l, board_corner_number, one_image_corners);
		bool found_r = findChessboardCorners(img_r, board_corner_number, one_image_corners);

		if (found_l == true && found_r == true) {	//If both image success

			cornerSubPix(img_l, one_image_corners, Size(5, 5), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 20, 0.01));
			all_image_corners_l.push_back(one_image_corners);
			
			cornerSubPix(img_r, one_image_corners, Size(5, 5), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 20, 0.01));
			all_image_corners_r.push_back(one_image_corners);

			//Draw the corners
			//drawChessboardCorners(img_l, board_corner_number, one_image_corners, found);

			//Show the image
			//imshow(file_list_l[image_counter], img_l);
			//waitKey(150);

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
			
		}
		else {	//If failed
			cout << "findChessboardCorners() failed." << endl;
		}
	}
	cout << "Find point completed." << endl;
}

void go_stereo_calibrate(vector<vector<Point3f>> &all_object_point, 
	vector<vector<Point2f>> &all_image_corners_l, vector<vector<Point2f>> &all_image_corners_r, 
	Mat &camera_matrix_l, Mat &distortion_coeffs_l, Mat &camera_matrix_r, Mat &distortion_coeffs_r,
	Size &image_size){

	Mat R, T, E, F;

	cout << "Start stereoCalibrate." << endl;
	double res = stereoCalibrate(all_object_point, all_image_corners_l, all_image_corners_r,
		camera_matrix_l, distortion_coeffs_l, camera_matrix_r, distortion_coeffs_r, 
		image_size, R, T, E, F,
		CALIB_USE_INTRINSIC_GUESS,
		TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));
	cout << "StereoCalibrate completed." << endl;
	
	//Save res
	cout << "Save result" << endl;
	FileStorage save_fs(g_save_path, FileStorage::WRITE);
	if (!save_fs.isOpened())
	{
		cerr << "Failed to open" << endl;
	}
	save_fs << "camera_matrix_l" << camera_matrix_l;
	save_fs << "distortion_coeffs_l" << distortion_coeffs_l;
	save_fs << "camera_matrix_r" << camera_matrix_r;
	save_fs << "distortion_coeffs_r" << distortion_coeffs_r;
	save_fs << "R" << R;
	save_fs << "T" << T;
	save_fs << "E" << R;
	save_fs << "F" << T;
	save_fs.release();
	cout << "Save result completed." << endl;

}


int main()
{
	Size image_size = Size(640, 480);
	vector<string> file_list_l, file_list_r;
	file_list_l = get_file_list(g_read_photo_list_path_l);
	file_list_r = get_file_list(g_read_photo_list_path_r);

	Mat camera_matrix_l, distortion_coeffs_l, camera_matrix_r, distortion_coeffs_r;
	get_camera_matrix_and_distCoeffs(camera_matrix_l, distortion_coeffs_l, 
		camera_matrix_r, distortion_coeffs_r);

	vector<vector<Point2f>> all_image_corners_l;
	vector<vector<Point2f>> all_image_corners_r;
	vector<vector<Point3f>> all_object_points;

	get_points(file_list_l, file_list_r, all_image_corners_l, all_image_corners_r, 
		all_object_points, image_size);

	go_stereo_calibrate(all_object_points, all_image_corners_l,
		all_image_corners_r, camera_matrix_l, distortion_coeffs_l, 
		camera_matrix_r, distortion_coeffs_r, image_size);

	return 0;
}
