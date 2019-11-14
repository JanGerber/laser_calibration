#ifndef ARM_TO_LASER_SCANNER_CALIBRATION_H
#define ARM_TO_LASER_SCANNER_CALIBRATION_H

#include <ros/ros.h>
#include <calibration_flat/ArmToLaserScannerCalibrationAction.h>
#include <actionlib/server/simple_action_server.h>
#include <string.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include <calibration_flat/LaserScanner.h>
#include <calibration_flat/Robot.h>
#include <Eigen/Dense>
#include <calibration_flat/Toolbox.h>
#include <calibration_flat/UserInput.h>
#include <Eigen/Geometry>
#include <math.h>
#include <calibration_flat/CalibrationObjectPoints.h>
#include <thread>


class ArmToLaserScannerCalibration {
public:
    ArmToLaserScannerCalibration();

    virtual ~ArmToLaserScannerCalibration();

    void executeCalibrationAction();

    void spin();

private:
    Eigen::Matrix4d runCalibration();

    void importParameter();

    void saveCalibrationMatrix(Eigen::Matrix4d calibrationMatrix);

    void addFrameCalibrationTarget(Eigen::Matrix4d homMatrixBaseTarget);

    ros::NodeHandle _nh;
    actionlib::SimpleActionServer<calibration_flat::ArmToLaserScannerCalibrationAction> _actionServer;
    calibration_flat::ArmToLaserScannerCalibrationFeedback feedback_;
    calibration_flat::ArmToLaserScannerCalibrationResult result_;
    Robot _robot;
    LaserScanner _laserScanner;
    UserInput _userInput;
    Eigen::Matrix4d
    calculateHomMatrixBaseObject(Eigen::Vector3d position1, Eigen::Vector3d position2, Eigen::Vector3d position3);

    bool autostart;
    bool initialCalibration;
    float widthCalibrationObject;
    float angleCalibrationObject;
    float nearDistance;
    float farDistance;
    float homePosX;
    float homePosY;
    float homePosZ;
    float pinLenght;
    float offsetAlphaZ;
    float ransacThreshold;
    float toleranceInlier;
    float transEeL;
    float offsetPreScan;
    float calibrationObjectOffsetZ;
    int numberOfScans;
    int numberOfIterRansacLS;
    std::string armBaseFrame;
    std::string armToolFrame;
    std::string handGuidingTopic;
    std::string calibrationPinFrame;
    std::string laserScanTopic;
    std::string calibrationObjectFrame;
    std::string targetDir;
    float deltaACalibrationObject;

    Eigen::Matrix4d getHomMatrixLaserScannerObject(CalibrationObjectPoints points);

    Eigen::Matrix4d getHomMatrixToolLaserScanner(Eigen::Matrix4d h_B_T, Eigen::Matrix4d h_B_O, Eigen::Matrix4d h_L_O);
};

#endif