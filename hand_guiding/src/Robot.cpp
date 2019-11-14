#include <hand_guiding/Robot.h>

#define MAX_RETRIES 3
#define RETRY_TIMEOUT 0.5


unsigned int robotId = 0;

Robot::Robot(ros::NodeHandle &nh) :
        _nh(nh),
        _moveGroupClient(nh, ""),
        _id(robotId++),
        _jointStateSet(false),
        _jointStateUpdated(false) {
    controlModeService.init("iiwa");
}

Robot::Robot(ros::NodeHandle &nh, const std::string &armBaseFrame, const std::string &armEndFrame,
             const std::string &armToolFrame,
             const std::string &calibrationPinFrame, const std::string &jointStatesTopic,
             const std::string &ikServiceTopic, const std::string &moveActionTopic, const std::string &moveGroupName, const float &pinLenght) :
        _nh(nh),
        _moveGroupClient(nh, moveActionTopic) {
    init(armBaseFrame, armEndFrame, armToolFrame, calibrationPinFrame, jointStatesTopic, ikServiceTopic,
         moveActionTopic, moveGroupName, pinLenght);
}

Robot::~Robot() {
}


Robot &Robot::Robot::operator=(const Robot &robot) {
    if (&robot == this) {
        return *this;
    }
    _nh = robot._nh;

    //_moveGroupClient = actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>(_nh, arm._moveActionTopic) ;
    _moveGroupClient.~SimpleActionClient(); // copy assignment does not work for this object :-/
    new(&_moveGroupClient) actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>(_nh, robot._moveActionTopic);


    init(robot.armBaseFrame, robot.armEndFrame, robot.armToolFrame, robot.calibrationPinFrame, robot._jointStatesTopic,
         robot._ikServiceTopic, robot._moveActionTopic, robot._moveGroupName, robot.pinLenght);
    return *this;
}

unsigned int
Robot::init(const std::string &armBaseFrame, const std::string &armEndFrame, const std::string &armToolFrame,
            const std::string &calibrationPinFrame, const std::string &jointStatesTopic,
            const std::string &ikServiceTopic, const std::string &moveActionTopic, const std::string &moveGroupName, const float &pinLenght) {
    this->armBaseFrame = armBaseFrame;
    this->armEndFrame = armEndFrame;
    this->armToolFrame = armToolFrame;
    this->calibrationPinFrame = calibrationPinFrame;
    this->pinLenght = pinLenght;

    _jointStatesTopic = jointStatesTopic;
    _ikServiceTopic = ikServiceTopic;
    _moveActionTopic = moveActionTopic;
    _moveGroupName = moveGroupName;
    _jointStateSet = false;
    _jointStateUpdated = false;

    _id = robotId++;
    _jointStateSub = _nh.subscribe(_jointStatesTopic, 1, &Robot::jointStateCallback, this);
    _ikClient = _nh.serviceClient<moveit_msgs::GetPositionIK>(_ikServiceTopic);

    return _id;
}

void Robot::jointStateCallback(const sensor_msgs::JointStatePtr &msg) {
    if (!_jointStateSet) {
        _jointStateSet = true;
    }

    _jointStateUpdated = true;
    _jointState = msg;
}


Eigen::Vector3d Robot::getPinPosition() {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform(armBaseFrame, armToolFrame, ros::Time(0), ros::Duration(1));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("[Robot] %s", ex.what());
        ros::Duration(1.0).sleep();
    }
    ROS_INFO_STREAM(
            "[ArmToLaserScannerCalibration] Pin Pos (X, Y, Z):                    ("
                    << transformStamped.transform.translation.x
                    << ", "
                    << transformStamped.transform.translation.y
                    << ", "
                    << transformStamped.transform.translation.z
                    << ") ");
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] Pose Test" <<transformStamped);
    Eigen::Vector3d translation =  Toolbox::toEigenVector(transformStamped.transform.translation);

    Eigen::Matrix3d rot = Toolbox::toEigenMatrix(transformStamped.transform.rotation);

    Eigen::Vector3d transPin(0,0,pinLenght);

    translation += rot * transPin;

    return translation;

}

void Robot::gravityCompensation(bool activate) {
    if (activate) {

        iiwa_msgs::JointQuantity joint_stiffnes;
        iiwa_msgs::JointQuantity joint_damping;

        float stiffnes = 10;
        float damping = 1;
        joint_stiffnes.a1 = 0; //0
        joint_stiffnes.a2 = 15; //25
        joint_stiffnes.a3 = 10; //10
        joint_stiffnes.a4 = 7; //7
        joint_stiffnes.a5 = 4; //4
        joint_stiffnes.a6 = 4; //0
        joint_stiffnes.a7 = 5; //0
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

Eigen::Matrix4d Robot::getHomMatrixBaseTool() {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform(armBaseFrame, armEndFrame, ros::Time(0), ros::Duration(1));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("[ArmToLaserScannerCalibration] %s", ex.what());
        ros::Duration(1.0).sleep();
    }
    return Toolbox::toEigen(transformStamped);
}

void Robot::moveToScanPosition(Eigen::Matrix4d h_B_O, float distanceY, float distanceZ, float offsetAlphaZ) {


    offsetAlphaZ = offsetAlphaZ / 180.0 * M_PI;

    Eigen::Matrix3d rotBO = h_B_O.block<3,3>(0,0);
    Eigen::Vector3d translationO(0, distanceY, distanceZ);
    Eigen::Vector3d translationB = rotBO * translationO;


    Eigen::Matrix4d h_S = h_B_O;

    Eigen::Vector3d xRot = h_S.block<3, 1>(0, 0) * -1;
    Eigen::Vector3d yRot = h_S.block<3, 1>(0, 2) * -1;
    Eigen::Vector3d zRot = h_S.block<3, 1>(0, 1) * -1;

    Eigen::Vector3d translation;
    translation = h_B_O.block<3, 1>(0, 3) + translationB;


    h_S.block<3, 1>(0, 0) = xRot;
    h_S.block<3, 1>(0, 1) = yRot;
    h_S.block<3, 1>(0, 2) = zRot;
    h_S.block<3, 1>(0, 3) = translation;

//    Eigen::Matrix4d h_OffsetAlpha;
//    h_OffsetAlpha.setIdentity();
//    h_OffsetAlpha(0,0) = cos(offsetAlphaZ);
//    h_OffsetAlpha(0,1) = sin(offsetAlphaZ);
//    h_OffsetAlpha(1,0) = -sin(offsetAlphaZ);
//    h_OffsetAlpha(1,1) = cos(offsetAlphaZ);


    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] Matrix h_S: \n" << h_S);

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = armBaseFrame;
    header.seq = 0;
    geometry_msgs::PoseStamped pose = Toolbox::toPoseStampedMsg(h_S, header);
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] Scan position: " << pose);
    bool targetReached = moveToPose(pose);
}

void Robot::moveToHomePosition(float x, float y, float z) {
    Eigen::Matrix4d homTransMatrix;
    homTransMatrix.setIdentity();
    homTransMatrix(0, 3) = x;
    homTransMatrix(1, 3) = y;
    homTransMatrix(2, 3) = z;


    Eigen::Matrix3d rot;
    rot << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    homTransMatrix.block<3, 3>(0, 0) = rot;


    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = armBaseFrame;
    header.seq = 0;
    geometry_msgs::PoseStamped pose = Toolbox::toPoseStampedMsg(homTransMatrix, header);
    ros::Duration(0.3).sleep();
    bool targetReached = moveToPose(pose);
    ros::Duration(1).sleep();
}

bool Robot::moveToPose(const geometry_msgs::PoseStamped &pose) {
    Eigen::VectorXf configuration = solveIk(pose);

    if (configuration.rows() <= 0) { // No IK solution found
        ROS_WARN_STREAM("[ArmToLaserScannerCalibration] Pose is out of reach!");
        return false;
    }

    return moveToConfiguration(configuration);
}

Eigen::VectorXf Robot::solveIk(const geometry_msgs::PoseStamped &pose) {
    ros::spinOnce();
    assert(_jointStateSet);

    moveit_msgs::GetPositionIK call;

    call.request.ik_request.group_name = _moveGroupName;
   // call.request.ik_request.ik_link_name     = armToolFrame ;
    call.request.ik_request.avoid_collisions = true;
    call.request.ik_request.pose_stamped = pose;

    call.request.ik_request.robot_state.joint_state = *_jointState;

    Eigen::VectorXf configuration;

    if (_ikClient.call(call)) {
        configuration.resize(_jointState->name.size());

        for (unsigned int i = 0; i < call.response.solution.joint_state.position.size() &&  i < _jointState->name.size(); i++) {
            configuration(i) = call.response.solution.joint_state.position[i];
        }

        ROS_INFO_STREAM("[ArmToLaserScannerCalibration] IK solution: (" << configuration.transpose() << ")");
    }

    return configuration;
}

bool Robot::moveToConfiguration(const Eigen::VectorXf &configuration) {
    assert(_jointStateSet);
    assert(_jointState->name.size() == configuration.rows());

    moveit_msgs::Constraints constraint;
    for (unsigned int i = 0; i < _jointState->name.size(); i++) {
        moveit_msgs::JointConstraint jc;
        jc.joint_name = _jointState->name[i];
        jc.position = configuration(i);
        jc.tolerance_above = 0.0001;
        jc.tolerance_below = 0.0001;
        jc.weight = 1.0;
        constraint.joint_constraints.push_back(jc);
    }

    moveit_msgs::MoveGroupGoal goal;
    goal.request.workspace_parameters.header.stamp = ros::Time::now();
    goal.request.workspace_parameters.header.frame_id = armBaseFrame;
    goal.request.workspace_parameters.min_corner.x = -1.f;
    goal.request.workspace_parameters.min_corner.y = -1.f;
    goal.request.workspace_parameters.min_corner.z = -1.f;
    goal.request.workspace_parameters.max_corner.x = 1.f;
    goal.request.workspace_parameters.max_corner.y = 1.f;
    goal.request.workspace_parameters.max_corner.z = 1.f;

    goal.request.start_state.is_diff = true; //true

    goal.request.goal_constraints.push_back(constraint);

    goal.request.group_name = _moveGroupName;
    goal.request.num_planning_attempts = 10;
    goal.request.allowed_planning_time = 20.0; //5
    goal.request.max_velocity_scaling_factor = 0.1; //1.0
    goal.request.max_acceleration_scaling_factor = 0.3; //1.0

    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.plan_only = false;
    goal.planning_options.look_around = false;
    goal.planning_options.look_around_attempts = 0;
    goal.planning_options.max_safe_execution_cost = 0.0;
    goal.planning_options.replan = false;
    goal.planning_options.replan_attempts = 0;
    goal.planning_options.replan_delay = 2.0;


    _moveGroupClient.sendGoal(goal);

    _moveGroupClient.waitForResult();
    moveit_msgs::MoveGroupResultConstPtr result = _moveGroupClient.getResult();

    if (result->error_code.val != result->error_code.SUCCESS) {
        ROS_ERROR_STREAM("[MoveItArm] MoveGroupAction failed: " << result->error_code.val);

        for (unsigned int i = 1; i <= MAX_RETRIES && result->error_code.val == result->error_code.CONTROL_FAILED; i++) {
            ROS_INFO_STREAM("[MoveItArm] Retrying... (" << i << "/" << MAX_RETRIES << ")");
            ros::Duration(RETRY_TIMEOUT).sleep();

            _moveGroupClient.sendGoal(goal);
            _moveGroupClient.waitForResult();
            result = _moveGroupClient.getResult();

            if (result->error_code.val == result->error_code.SUCCESS) {
                ROS_INFO_STREAM("[MoveItArm] Retry success!");
                return true;
            } else {
                ROS_ERROR_STREAM("[MoveItArm] MoveGroupAction failed: " << result->error_code.val);
            }
        }

        return false;
    }

    return true;
}







