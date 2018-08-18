在代码文件code.cpp中有三个路径：
1. g_read_camera_matrix_path：存储了相机的相机矩阵与畸变参数
2. g_read_photo_list_path：存储了待处理图像文件列表
3. g_save_path：指向校正后图片结果输出位置

在运行代码时将camera_matrix_l.xml，left_photo_list.xml和待处理图片放入工程目录。
如果没有修改结果输出路径，矫正结果会输出至工程目录
