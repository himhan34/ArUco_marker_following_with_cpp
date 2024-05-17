#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

class ImageConverter {
public:
    ImageConverter()
        : it(nh) {
        // Publisher를 초기화합니다.
        image_pub = nh.advertise<sensor_msgs::Image>("cvbridge_image", 1);
        ROS_INFO("Published to /camera/image");

        // Subscriber를 초기화합니다.
        image_sub = it.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCallback, this);
        ROS_INFO("Subscribed to /camera/image");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        ROS_INFO("Callback triggered");
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::CvBridgeException& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        Mat cv_image = cv_ptr->image;
        imshow("Image window", cv_image);
        waitKey(3);

        try {
            image_pub.publish(cv_ptr->toImageMsg());
        } catch (cv_bridge::CvBridgeException& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;
    image_transport::Subscriber image_sub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    destroyAllWindows();
    return 0;
}
