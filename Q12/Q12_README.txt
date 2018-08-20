程序功能介绍：
  本程序会根据输入的左相机相机矩阵与畸变参数，右相机相机矩阵与畸变参数，于左右相机拍摄的图片对，进行立体标定
  输出左右相机相机矩阵与畸变参数，左右相机转换参数R，T，本征矩阵E，基础矩阵F。

程序输入输出介绍：
  在代码文件code.cpp中有5个路径：
  1. g_read_camera_matrix_path_l:存储了左相机的相机矩阵与14维畸变参数
  2. g_read_camera_matrix_path_r:存储了右相机的相机矩阵与14维畸变参数
  3. g_read_photo_list_path_l:存储了待处理的左相机拍摄图片列表
  4. g_read_photo_list_path_r:存储了待处理的右相机拍摄图片列表
  5. g_save_path:指向立体标定结果输出位置

  运行时将camera_matrix_r.xml，camera_matrix_l.xml，left_photo_list.xml，right_photo_list.xml和对应的待处理图片放入工程文件夹。
  立体标定结果会输出至工程文件夹的stereoCalibrate_res.xml中。
