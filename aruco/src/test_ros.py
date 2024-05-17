#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

using namespace cv;
using namespace std;

VideoCapture cap(0);  // Get the camera source
int count = 1;  // Count images

void track(Mat matrix_coefficients, Mat distortion_coefficients) {
    double width = cap.get(CAP_PROP_FRAME_WIDTH);
    double height = cap.get(CAP_PROP_FRAME_HEIGHT);

    cout << "Matrix Coefficients: " << matrix_coefficients << endl;

    while (true) {
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;
        count++;
        cout << count << "th image" << endl;

        // Convert frame to grayscale
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Use 5x5 dictionary to find markers
        Ptr<aruco::Dictionary> aruco_dict = aruco::getPredefinedDictionary(aruco::DICT_5X5_250);
        Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();

        // Detect markers
        vector<int> ids;
        vector<vector<Point2f>> corners;
        aruco::detectMarkers(gray, aruco_dict, corners, ids, parameters);

        cout << "IDs: " << ids << endl;
        if (ids.empty()) {
            cout << "Marker undetected" << endl;
        } else {
            cout << "Marker detected" << endl;
            cout << "Number of markers: " << ids.size() << endl;

            for (int i = 0; i < ids.size(); i++) {
                Vec3d rvec, tvec;
                aruco::estimatePoseSingleMarkers(corners[i], 0.1, matrix_coefficients, distortion_coefficients, rvec, tvec);
                cout << "rvec: " << rvec << endl;
                cout << "tvec: " << tvec << endl;

                aruco::drawDetectedMarkers(frame, corners, ids);
                aruco::drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.05);

                cout << "tvec[0]: " << tvec[0] << endl;
                cout << "Width: " << width << endl;
                cout << "Height: " << height << endl;

                if (tvec[0] > 0.01) {
                    arrowedLine(frame, Point(490, 240), Point(590, 240), Scalar(138, 43, 226), 3);
                } else if (tvec[0] < -0.01) {
                    arrowedLine(frame, Point(150, 240), Point(50, 240), Scalar(138, 43, 226), 3);
                }

                if (tvec[0] > 0 && tvec[0] < 0.02) {
                    cout << "-------------------------------------------------------" << endl;
                    break;
                }
            }
        }

        cout << "Show image" << endl;
        imshow("frame", frame);

        // Wait for 3 milliseconds for an interaction
        char key = (char)waitKey(3);
        if (key == 'q') break;
    }
}

int main(int argc, char** argv) {
    Mat matrix_coefficients = (Mat_<double>(3,3) << 493.50028035371434, 0., 318.89472537064762, 0., 497.52379240241839, 233.23580951832940, 0., 0., 1.);
    Mat distortion_coefficients = (Mat_<double>(1,5) << 0.13514513045692980, -0.48663060594638929, 0.00063572242938879548, 0.00056972282484044220, 0.54433200932025450);

    cout << "Matrix Coefficients: " << matrix_coefficients << endl;
    track(matrix_coefficients, distortion_coefficients);
    cout << "Matrix Coefficients: " << matrix_coefficients << endl;

    cap.release();
    destroyAllWindows();

    return 0;
}
