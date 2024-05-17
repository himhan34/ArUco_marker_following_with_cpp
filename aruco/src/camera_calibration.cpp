#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

// termination criteria
TermCriteria criteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001);

void calibrate(const string& dirpath, const string& prefix, const string& image_format, float square_size, int width, int height,
               Mat& camera_matrix, Mat& dist_coeffs) {
    // prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    vector<Point3f> objp;
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            objp.emplace_back(j * square_size, i * square_size, 0);
        }
    }

    vector<vector<Point3f>> objpoints; // 3d point in real world space
    vector<vector<Point2f>> imgpoints; // 2d points in image plane

    for (const auto& entry : fs::directory_iterator(dirpath)) {
        string fname = entry.path().string();
        if (fname.find(prefix) != string::npos && fname.find(image_format) != string::npos) {
            Mat img = imread(fname);
            Mat gray;
            cvtColor(img, gray, COLOR_BGR2GRAY);

            // Find the chess board corners
            vector<Point2f> corners;
            bool ret = findChessboardCorners(gray, Size(width, height), corners);

            // If found, add object points, image points (after refining them)
            if (ret) {
                objpoints.push_back(objp);

                cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), criteria);
                imgpoints.push_back(corners);

                // Draw and display the corners
                drawChessboardCorners(img, Size(width, height), corners, ret);
                imshow("Chessboard Corners", img);
                waitKey(500); // Display for 500 ms
            }
        }
    }

    Mat rvecs, tvecs;
    calibrateCamera(objpoints, imgpoints, Size(gray.rows, gray.cols), camera_matrix, dist_coeffs, rvecs, tvecs);
}

void save_coefficients(const Mat& camera_matrix, const Mat& dist_coeffs, const string& path) {
    FileStorage fs(path, FileStorage::WRITE);
    fs << "K" << camera_matrix;
    fs << "D" << dist_coeffs;
    fs.release();
}

void load_coefficients(const string& path, Mat& camera_matrix, Mat& dist_coeffs) {
    FileStorage fs(path, FileStorage::READ);
    fs["K"] >> camera_matrix;
    fs["D"] >> dist_coeffs;
    fs.release();
}

int main(int argc, char** argv) {
    CommandLineParser parser(argc, argv, "{image_dir||}{image_format||}{prefix||}{square_size|1.0|}{width|9|}{height|6|}{save_file||}");
    parser.about("Camera calibration");

    if (!parser.has("image_dir") || !parser.has("image_format") || !parser.has("prefix") || !parser.has("save_file")) {
        parser.printMessage();
        return -1;
    }

    string image_dir = parser.get<string>("image_dir");
    string image_format = parser.get<string>("image_format");
    string prefix = parser.get<string>("prefix");
    float square_size = parser.get<float>("square_size");
    int width = parser.get<int>("width");
    int height = parser.get<int>("height");
    string save_file = parser.get<string>("save_file");

    Mat camera_matrix, dist_coeffs;
    calibrate(image_dir, prefix, image_format, square_size, width, height, camera_matrix, dist_coeffs);
    save_coefficients(camera_matrix, dist_coeffs, save_file);

    cout << "Calibration is finished." << endl;
    return 0;
}
