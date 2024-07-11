#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
using namespace cv;
using namespace std;

int main()
{
    // set the column and row of corners
    int row = 6, col = 8;

    //length of each block is 24.5mm
    float distance = 24.5e-8; // the distance between two adjacent corners

    // A vector used for storing the coordinate in 3-D space
    vector<Point3f> objp;
    for (int j = 0; j < col; ++j)
    {
        for (int i = 0; i < row; ++i)
            objp.push_back(Point3f(i * distance, j * distance, 0));
    }

    // store the corner coordinate of all the pictures
    vector<vector<Point3f>> objpoints; // 3-D points in real world space
    vector<vector<Point2f>> imgpoints; // 2-D points in image plane

    // go through every picture to obtain the corner coordinate of the chessboard
    vector<String> filename;
    glob("chessboard*.jpg", filename, false);
    for (size_t i = 0; i < filename.size(); ++i)
    {
        Mat img = imread(filename[i]);
        Mat gray;
        cvtColor(img, gray, COLOR_BGR2GRAY);

        // find the corners of the chessboard
        vector<Point2f> corners;
        bool success_find = findChessboardCorners(gray, Size(row, col), corners);

        // if find the corners successfully, store the corner point info
        if (success_find)
        {
            objpoints.push_back(objp);
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            imgpoints.push_back(corners);

            // show the info of the corners
            drawChessboardCorners(img, Size(row, col), corners, success_find);
            imshow("corners", img);
            waitKey(1000);
        }
    }

    Mat img = imread(filename[0]);
    Size img_shape = img.size();

    // calibration
    Mat camera_matrix, dist_coeffs; // intinsic(3*3) and distortion matrix
    vector<Mat> rvecs, trecs;       // rotation and moving vector,both 3*1
    calibrateCamera(objpoints, imgpoints, img_shape, camera_matrix, dist_coeffs, rvecs, trecs);

    cout << "camera_matrix" << camera_matrix << endl;
    cout << "dist_coeffs" << dist_coeffs << endl;
    return 0;
}
