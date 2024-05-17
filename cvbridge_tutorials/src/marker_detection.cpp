#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>

using namespace cv;
using namespace std;

class ImageConverter {
public:
    ImageConverter() {
        pose_pub = nh.advertise<geometry_msgs::Pose>("pose", 1000);
        image_pub = nh.advertise<sensor_msgs::Image>("image", 1000);
        image_sub = nh.subscribe("/camera/color/image_raw", 1000, &ImageConverter::callback, this);
        loadCameraCalibration("/home/fish/fish_ws/src/aruco/src/image_web/ost.yaml");
    }

    void callback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::CvBridgeException& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        Mat cv_image = cv_ptr->image;
        Mat cv_image_resized;
        resize(cv_image, cv_image_resized, Size(10, 10), 0, 0, INTER_AREA);
        
        Mat gray, gray_resized;
        cvtColor(cv_image, gray, COLOR_BGR2GRAY);
        cvtColor(cv_image_resized, gray_resized, COLOR_BGR2GRAY);

        Scalar pixelsum = sum(gray_resized);
        ROS_INFO_STREAM("Sum: " << pixelsum);

        Ptr<aruco::Dictionary> aruco_dict = aruco::getPredefinedDictionary(aruco::DICT_7X7_1000);
        Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
        vector<int> ids;
        vector<vector<Point2f>> corners;
        aruco::detectMarkers(gray, aruco_dict, corners, ids, parameters);

        geometry_msgs::Pose p;

        if (!ids.empty()) {
            ROS_INFO_STREAM("Marker ID: " << ids);
            for (size_t i = 0; i < ids.size(); i++) {
                Vec3d rvec, tvec;
                aruco::estimatePoseSingleMarkers(corners[i], 0.1, matrix_coefficients, distortion_coefficients, rvec, tvec);

                Mat R;
                Rodrigues(rvec, R);
                Mat tvec_mat = Mat(tvec).reshape(1, 3);

                Mat t;
                hconcat(R, tvec_mat, t);
                Mat bottom_row = Mat::zeros(1, 4, CV_64F);
                bottom_row.at<double>(0, 3) = 1.0;
                t.push_back(bottom_row);

                Mat tt = t.inv();
                Mat ct = tt(Rect(3, 0, 1, 3)).clone();

                aruco::drawDetectedMarkers(cv_image, corners, ids);
                aruco::drawAxis(cv_image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.05);

                p.position.x = tvec[0];
                p.position.y = tvec[1];
                p.position.z = tvec[2];
                p.orientation.x = 0.0;
                p.orientation.y = 0.0;
                p.orientation.z = 0.0;
                p.orientation.w = 1.0;
            }
        }

        imshow("Image window", cv_image);
        waitKey(3);

        try {
            pose_pub.publish(p);
            sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
            image_pub.publish(out_msg);
        } catch (cv_bridge::CvBridgeException& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pose_pub;
    ros::Publisher image_pub;
    ros::Subscriber image_sub;

    Mat matrix_coefficients;
    Mat distortion_coefficients;

    void loadCameraCalibration(const string& filename) {
        YAML::Node config = YAML::LoadFile(filename);
        YAML::Node K = config["K"];
        YAML::Node D = config["D"];

        vector<double> kdata = K["kdata"].as<vector<double>>();
        vector<double> ddata = D["ddata"].as<vector<double>>();

        matrix_coefficients = Mat(3, 3, CV_64F, kdata.data()).clone();
        distortion_coefficients = Mat(1, 5, CV_64F, ddata.data()).clone();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    destroyAllWindows();
    return 0;
}
