#include <ros/ros.h>
#include <calibration_flat/ArmToLaserScannerCalibrationNode.h>

int main(int argc, char *argv[]) {
    // Init ROS
    ros::init(argc, argv, "arm_to_laser_scanner_calibration");

    ArmToLaserScannerCalibration node;
    node.spin();
}
