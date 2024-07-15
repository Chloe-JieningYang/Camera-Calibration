#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <fstream>
#include <sstream>
#include <iomanip>

using namespace std;
using namespace cv;

#define DATA_NUM 19

double string2double(const string &str)
{
    istringstream iss(str);
    double num;
    iss >> num;
    return num;
}

int main()
{
    // read csv file to get 3-D object points
    string csvname = "gps.CSV";
    ifstream csv_in(csvname, ios::in);
    vector<vector<double>> obj_csv_data;

    if (!csv_in.is_open())
    {
        cout << "Fail to open csv file" << endl;
        return -1;
    }

    string lineStr;
    vector<vector<string>> strArray;
    while (getline(csv_in, lineStr))
    {
        stringstream ss(lineStr);
        string str;
        vector<string> saperated_str;
        while (getline(ss, str, ','))
        {
            saperated_str.push_back(str);
        }
        strArray.push_back(saperated_str);
    }
    // turn string matrix to double matrix
    for (size_t i = 0; i < strArray.size(); ++i)
    {
        vector<double> temp;
        for (int j = 0; j < 3; ++j)
            temp.push_back(string2double(strArray[i][j]));
        obj_csv_data.push_back(temp);
    }
    csv_in.close();
    /*
    for (size_t i = 0; i < obj_csv_data.size(); ++i)
    {
        for (int j = 0; j < 3; ++j)
            cout << obj_csv_data[i][j]<<" ";
        cout << endl;
    }
    */

    // read txt file to get 2-D image points
    string txtname="images.txt";
    vector<vector<double>> img_txt_data;

    ifstream txt_in(txtname, ios::in);
    if(!txt_in.is_open())
    {
        cerr<<"Fail to open txt file"<<endl;
        return -1;
    }
    string line;
    while (getline(txt_in, line)) {
        stringstream ss(line);
        string x_str, y_str;

        // 使用逗号分隔符读取每个坐标
        if (getline(ss, x_str, ',') && getline(ss, y_str)) {
            // 将字符串转换为双精度数并存储到向量中
            double x = stod(x_str);
            double y = stod(y_str);
            img_txt_data.push_back({x, y});
        }
    }

    txt_in.close();

    /*
    use epnp algorithm to find out the extrinsic
    */
    vector<Point2d> image_points;
    for (size_t i = 0; i < img_txt_data.size(); i++)
    {
        image_points.push_back(Point2d(img_txt_data[i][0], img_txt_data[i][1]));
    }

    vector<Point3d> obj_points;
    for (size_t i = 0; i < obj_csv_data.size(); i++)
    {
        obj_points.push_back(Point3d(obj_csv_data[i][0], obj_csv_data[i][1], obj_csv_data[i][2]));
    }

    Mat camera_intrinsic = (Mat_<double>(3, 3) << 4324.243366089897, 0, 959.4999999167276,
                            0, 4621.143167610721, 539.499999881995,
                            0, 0, 1);
    Mat dist_coeffs = (Mat_<double>(5, 1) << -8.795233965715913e-06, 0, -0.00146416259710504,
                       0.001223582419864574, 0);

    // rotation vector
    Mat rotation_vector;
    // translation_vector
    Mat translation_vector;

    // Epnp solution
    solvePnP(obj_points, image_points, camera_intrinsic, dist_coeffs,
             rotation_vector, translation_vector, false, CV_EPNP);

    cout << "Rotation Vector " << endl
         << rotation_vector << endl;
    cout << "Translation Vetcor" << endl
         << translation_vector << endl;

    Mat rotation_matrix;
    Rodrigues(rotation_vector, rotation_matrix);
    cout << "Rotation Matrix" << endl
         << rotation_matrix << endl;

    Mat extrinsic=Mat::zeros(3,4,CV_64F);
    Mat sub_ex=extrinsic.colRange(0,3);
    rotation_matrix.copyTo(sub_ex);

    sub_ex=extrinsic.col(3);
    translation_vector.copyTo(sub_ex);

    cout<<"extrinsic:"<<extrinsic<<endl;

    Mat res=camera_intrinsic*extrinsic;
    cout<<"res:"<<res<<endl;
    
    Mat test_wd_point=(Mat_<double>(4, 1) << 31.291103738,121.205471734,16.734,1);

    // 将2D图像坐标转换为去畸变的图像坐标
    vector<Point2d> undistorted_points;
    undistortPoints(image_points, undistorted_points, camera_intrinsic, dist_coeffs);

    // 计算归一化图像坐标
    vector<Point2d> normalized_points;
    Mat K_inv = camera_intrinsic.inv();
    for (const auto& pt : undistorted_points) {
        //homogeneous image coordinate: [u,v,1] without coeffient s
        Mat pt_homogeneous = (Mat_<double>(3, 1) << pt.x, pt.y, 1);
        Mat normalized_pt = K_inv * pt_homogeneous;
        normalized_points.push_back(Point2d(normalized_pt.at<double>(0, 0), normalized_pt.at<double>(1, 0)));
    }

    // 使用外参矩阵将归一化图像坐标转换为相机坐标系中的3D点
    vector<Point3d> cam_points;
    for (const auto& pt : normalized_points) {
        Mat normalized_point = (Mat_<double>(3, 1) << pt.x, pt.y, 1);
        Mat cam_point = rotation_matrix.inv() * (normalized_point - translation_vector);
        cam_points.push_back(Point3d(cam_point.at<double>(0, 0), cam_point.at<double>(1, 0), cam_point.at<double>(2, 0)));
    }

        // 设置输出精度
    cout << fixed << setprecision(6);
    
    // 输出相机坐标系中的3D点
    for (const auto& point : cam_points) {
        cout << "X: " << point.x << ", Y: " << point.y << ", Z: " << point.z << endl;
    }

    return 0;
}
