#include <ros/ros.h>
#include "hand_guiding/Robot.h"
#include <hand_guiding/HandGuidingAction.h>
#include <actionlib/server/simple_action_server.h>
#include <iiwa_msgs/ConfigureControlMode.h>
#include <iiwa_ros/service/control_mode.hpp>
#include <thread>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <iiwa_msgs/JointPosition.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

class HandGuidingAction {
public:

    HandGuidingAction(std::string name) :
            as_(nh_, name, false),
            action_name_(name){
        //register the goal and feeback callbacks
        controlModeService.init("iiwa");
        handGuidingActivated = false;
        newGoal = false;
        buttonPressed= false;
        ros::Duration(3).sleep();
        as_.registerGoalCallback(boost::bind(&HandGuidingAction::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&HandGuidingAction::preemptCB, this));
        sub_ = nh_.subscribe("/iiwa/joint_states", 1, &HandGuidingAction::handGuidingRoutine, this);
        sub2_ = nh_.subscribe("/iiwa/state/MFButtonState", 1, &HandGuidingAction::activationButton, this);
        pub_ = nh_.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1000);
        as_.start();
    }

    ~HandGuidingAction(void) {
    }

    void goalCB() {
        // reset helper variables
        ROS_INFO_STREAM("new Goal");
        handGuidingActivated = as_.acceptNewGoal()->activateHandGuiding;
        newGoal = true;

    }

    void preemptCB() {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
    }


    void handGuidingRoutine(const sensor_msgs::JointStateConstPtr &joints) {
        if (newGoal) {
            newGoal = false;
            setMode();
            as_.setSucceeded();
        }
        if(buttonChanged){
            setMode();
            buttonChanged = false;
        }
        if (handGuidingActivated) {
            iiwa_msgs::JointPosition jointPosition;

            jointPosition.position.a1 = joints->position[0];
            jointPosition.position.a2 = joints->position[1];
            jointPosition.position.a3 = joints->position[2];
            jointPosition.position.a4 = joints->position[3];
            jointPosition.position.a5 = joints->position[4];
            jointPosition.position.a6 = joints->position[5];
            jointPosition.position.a7 = joints->position[6];

            pub_.publish(jointPosition);
        }

    }

    void setMode() {
        if (handGuidingActivated && buttonPressed) {
            iiwa_msgs::JointQuantity joint_stiffnes;
            iiwa_msgs::JointQuantity joint_damping;

            float stiffnes = 10;
            float damping = 0.2;
            joint_stiffnes.a1 = 0; //0
            joint_stiffnes.a2 = 0; //25
            joint_stiffnes.a3 = 0; //10
            joint_stiffnes.a4 = 0; //7
            joint_stiffnes.a5 = 0; //4
            joint_stiffnes.a6 = 0; //0
            joint_stiffnes.a7 = 0; //0
            joint_damping.a1 = damping;
            joint_damping.a2 = damping;
            joint_damping.a3 = damping;
            joint_damping.a4 = damping;
            joint_damping.a5 = damping;
            joint_damping.a6 = damping;
            joint_damping.a7 = damping;
            controlModeService.setJointImpedanceMode(joint_stiffnes, joint_damping);
        } else {
            controlModeService.setPositionControlMode();
        }
    }

    void activationButton(const std_msgs::BoolConstPtr &buttonActive){
        if(buttonPressed != buttonActive->data){
            buttonPressed = buttonActive->data;
            buttonChanged = true;
        }

    }


protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<hand_guiding::HandGuidingAction> as_;
    ros::Publisher pub_;
    std::string action_name_;
    bool handGuidingActivated;
    bool newGoal;
    bool buttonPressed;
    bool buttonChanged;
    hand_guiding::HandGuidingFeedback feedback_;
    hand_guiding::HandGuidingResult result_;
    iiwa_ros::service::ControlModeService controlModeService;
    ros::Subscriber sub_;
    ros::Subscriber sub2_;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "hand_guiding_node");

    HandGuidingAction handGuiding(ros::this_node::getName());
    ros::spin();

    return 0;
}