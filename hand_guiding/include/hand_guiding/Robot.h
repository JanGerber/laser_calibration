#ifndef ROBOT_H
#define ROBOT_H


#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <hand_guiding/Toolbox.h>
#include <iiwa_msgs/ConfigureControlMode.h>
#include <iiwa_ros/service/control_mode.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Header.h>


class Robot {
public:
    Robot(ros::NodeHandle &nh);

    Robot(ros::NodeHandle &nh, const std::string &armBaseFrame, const std::string &armEndFrame,
          const std::string &armToolFrame,
          const std::string &calibrationPinFrame, const std::string &jointStatesTopic,
          const std::string &ikServiceTopic, const std::string &moveActionTopic,
          const std::string &moveGroupName, const float &pinLenght);

    virtual ~Robot();

    Robot &operator=(const Robot &robot);

    Eigen::Vector3d getPinPosition();

    void gravityCompensation(bool activate);

    void moveToScanPosition(Eigen::Matrix4d h_B_O, float distanceY, float distanceZ, float offsetAlphaZ);

    void moveToHomePosition(float x, float y, float z);

    void jointStateCallback(const sensor_msgs::JointStatePtr &msg);
    //void updatePosition(std::atomic_bool stop);


    Eigen::Matrix4d getHomMatrixBaseTool();

private:
    bool moveToPose(const geometry_msgs::PoseStamped &pose);

    Eigen::VectorXf solveIk(const geometry_msgs::PoseStamped &pose);

    bool moveToConfiguration(const Eigen::VectorXf &configuration);

    unsigned int init(const std::string &armBaseFrame, const std::string &armEndFrame, const std::string &armToolFrame,
                      const std::string &calibrationPinFrame, const std::string &jointStatesTopic,
                      const std::string &ikServiceTopic, const std::string &moveActionTopic,
                      const std::string &moveGroupName, const float &pinLenght);

    ros::NodeHandle &_nh;
    iiwa_ros::service::ControlModeService controlModeService;
    ros::ServiceClient _ikClient;
    actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> _moveGroupClient;
    ros::Subscriber _jointStateSub;
    std::string armBaseFrame;
    std::string armEndFrame;
    std::string armToolFrame;
    std::string calibrationPinFrame;
    float pinLenght;
    unsigned int _id;


    std::string _ikServiceTopic;
    std::string _moveActionTopic;
    std::string _moveGroupName;
    std::string _jointStatesTopic;

    bool _jointStateSet;
    bool _jointStateUpdated;
    sensor_msgs::JointStatePtr _jointState;


};

#endif