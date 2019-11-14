#ifndef ROBOT_H
#define ROBOT_H


#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <calibration_flat/Toolbox.h>
#include <iiwa_msgs/ConfigureControlMode.h>
#include <iiwa_ros/service/control_mode.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Header.h>
#include <hand_guiding/HandGuidingAction.h>
#include <iiwa_msgs/MoveAlongSplineGoal.h>
#include <iiwa_msgs/MoveAlongSplineAction.h>
#include <iiwa_msgs/SetPTPCartesianSpeedLimits.h>
#include <iiwa_msgs/SetEndpointFrame.h>

class Robot {
public:
    Robot(ros::NodeHandle &nh);

    Robot(ros::NodeHandle &nh, const std::string &armBaseFrame,
          const std::string &armToolFrame, const std::string &calibrationPinFrame, const float &pinLenght,
          const std::string &handGuidingTopic, const std::string &moveAlongSplineTopic, const float &transEeL);

    virtual ~Robot();

    Robot &operator=(const Robot &robot);

    Eigen::Vector3d getPinPosition();

    void gravityCompensation(bool activate);

    void moveToScanPosition(Eigen::Matrix4d h_B_O, float distanceY, float distanceZ, float offsetAlphaZ);

    void moveToHomePosition(float x, float y, float z);

    bool setEndpointFrame(std::string frameId);

    bool setPTPCartesianSpeedLimits(double maxCartesianVelocity);

    Eigen::Matrix4d getHomMatrixBaseTool();

    void getHomMatrixLaserTool();

private:
    void moveToPose(Eigen::Matrix4d h_S);

    unsigned int init(const std::string &armBaseFrame, const std::string &armToolFrame,
                      const std::string &calibrationPinFrame, const float &pinLenght,
                      const std::string &handGuidingTopic,
                      const std::string &moveAlongSplineTopic,
                      const float &transEeL);

    ros::NodeHandle &_nh;
    actionlib::SimpleActionClient<hand_guiding::HandGuidingAction> _handGuidingClient;
    actionlib::SimpleActionClient<iiwa_msgs::MoveAlongSplineAction> splineMotionClient;

    std::string armBaseFrame;
    std::string armToolFrame;
    std::string calibrationPinFrame;
    std::string handGuidingTopic;
    std::string moveAlongSplineTopic;
    float pinLenght;
    float transEeL;
    unsigned int _id;
};

#endif