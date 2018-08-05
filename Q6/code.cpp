#include <iostream> 
#include <string>
#include <fstream>
#include <io.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc.hpp> 

/*Created by yangandong*/

using namespace cv;
using namespace std;

vector<string> get_file_list() {
	vector<string> file_list;
	const string file_name = "../../XML/left_photo_list.xml";

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

void cal_calibratecamera(vector<string> &file_list, Mat &cameraMatrix,
	Mat &distCoeffs, vector<Mat> &rvecsMat, vector<Mat> &tvecsMat){
	cout << "Find chessboard corners" << endl;
	int image_counter = 0;
	Size square_size = Size(50, 50); //The size of a square, in mm 
	Size image_size;
	Size board_corner_number = Size(9, 6);
	vector<Point2f> one_image_corners;
	vector<vector<Point2f>> all_image_corners;

	for (image_counter = 0; image_counter < file_list.size(); image_counter++) {
		Mat img = imread(file_list[image_counter], CV_LOAD_IMAGE_GRAYSCALE);
		image_size.width = img.cols;
		image_size.height = img.rows;

		bool found = findChessboardCorners(img, board_corner_number, one_image_corners,
			CV_CALIB_CB_ADAPTIVE_THRESH);

		if (found == 0) {	//If failed
			cout << "findChessboardCorners() failed, image path:" << file_list[image_counter] << endl;
		}
		else {	//If success
			cornerSubPix(img, one_image_corners, Size(11, 11), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 20, 0.01));

			all_image_corners.push_back(one_image_corners);

			//Draw the corners
			drawChessboardCorners(img, board_corner_number, one_image_corners, found);

			//Show the image
			imshow(file_list[image_counter], img);
			waitKey(150);
		}
	}
	cout << "Find chessboard corners success." << endl;

	//Create the 3D coordinate for chessboard
	cout << "calibrateCamera" << endl;
	vector<vector<Point3f>> all_object_points;
	for (image_counter = 0; image_counter < file_list.size(); image_counter++) {
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

	//calibrate camera
	double co = calibrateCamera(all_object_points, all_image_corners, image_size,
		cameraMatrix, distCoeffs,
		rvecsMat, tvecsMat, CV_CALIB_FIX_K3);
	cout << "calibrateCamera success." << endl;

	//Save camera matrix
	cout << "save camera matrix and distCoeffs matrix" << endl;
	FileStorage fs("F:/learn/baoyan/Stereo/camera_matrix.xml", FileStorage::WRITE);
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
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));        // Intrinsic matrix
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));          // distortion factor[k1,k2,k3,p1,p2]
	vector<Mat> rvecsMat;                                          // Extrinsic parameters R
	vector<Mat> tvecsMat;										   // Extrinsic parameters T
	
	vector<string> file_list = get_file_list();
	cal_calibratecamera(file_list, cameraMatrix, distCoeffs, rvecsMat, tvecsMat);

	return 0;
}
