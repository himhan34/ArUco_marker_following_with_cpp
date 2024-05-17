#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <iostream>

using namespace std;

class CmdPose {
public:
    CmdPose() {
        vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
        pose_sub = nh.subscribe("pose", 1000, &CmdPose::callback, this);
    }

    void callback(const geometry_msgs::Pose::ConstPtr& pose) {
        double angular = atan2(pose->position.x, pose->position.z);
        double linear = 0.35 * sqrt(pose->position.x * pose->position.x + pose->position.z * pose->position.z);

        cout << "angular: " << angular << endl;
        cout << "linear: " << linear << endl;

        if (linear < 0.1) {
            linear = 0.0;
        } else if (linear > 0.18) {
            linear = linear * 1.5;
        }
        if (angular < 0.0) {
            angular = angular * 1.5;
        } else if (angular > 0.0) {
            angular = angular * 1.5;
        }

        geometry_msgs::Twist cmd;
        cmd.linear.x = linear;
        cmd.angular.z = -angular;

        try {
            vel_pub.publish(cmd);
        } catch (ros::Exception& e) {
            ROS_ERROR("Error publishing velocity: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::Subscriber pose_sub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd_pose");
    CmdPose cp;

    try {
        ros::spin();
    } catch (ros::Exception& e) {
        ROS_ERROR("Exception: %s", e.what());
    }

    return 0;
}
