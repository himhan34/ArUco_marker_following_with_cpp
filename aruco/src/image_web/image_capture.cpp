#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    VideoCapture cam(0);

    if (!cam.isOpened()) {
        cerr << "Error: Could not open camera" << endl;
        return -1;
    }

    namedWindow("test", WINDOW_AUTOSIZE);

    int img_counter = 0;

    while (true) {
        Mat frame;
        bool ret = cam.read(frame);

        if (!ret) {
            cout << "Failed to grab frame" << endl;
            break;
        }

        imshow("test", frame);

        int k = waitKey(1);
        if (k % 256 == 27) {
            // ESC pressed
            cout << "Escape hit, closing..." << endl;
            break;
        } else if (k % 256 == 32) {
            // SPACE pressed
            string img_name = "opencv_frame_" + to_string(img_counter) + ".png";
            imwrite(img_name, frame);
            cout << img_name << " written!" << endl;
            img_counter++;
        }
    }

    cam.release();
    destroyAllWindows();

    return 0;
}
