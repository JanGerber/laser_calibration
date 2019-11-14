#include <calibration_flat/Robot.h>

#define MAX_RETRIES 3
#define RETRY_TIMEOUT 0.5


unsigned int robotId = 0;

Robot::Robot(ros::NodeHandle &nh) :
        _nh(nh),
        _id(robotId++),
        _handGuidingClient(nh, "/hand_guiding_node"),
        splineMotionClient(nh, "/iiwa/action/move_along_spline", true) {
}

Robot::Robot(ros::NodeHandle &nh, const std::string &armBaseFrame,
             const std::string &armToolFrame, const std::string &calibrationPinFrame, const float &pinLenght,
             const std::string &handGuidingTopic, const std::string &moveAlongSplineTopic, const float &transEeL) :
        _nh(nh),
        _handGuidingClient(nh, handGuidingTopic),
        splineMotionClient(nh, moveAlongSplineTopic, true) {
    init(armBaseFrame, armToolFrame, calibrationPinFrame, pinLenght, handGuidingTopic,
         moveAlongSplineTopic, transEeL);
}

Robot::~Robot() {
}


Robot &Robot::Robot::operator=(const Robot &robot) {
    if (&robot == this) {
        return *this;
    }
    _nh = robot._nh;
    _handGuidingClient.~SimpleActionClient(); // copy assignment does not work for this object :-/
    splineMotionClient.~SimpleActionClient(); // copy assignment does not work for this object :-/
    new(&_handGuidingClient) actionlib::SimpleActionClient<hand_guiding::HandGuidingAction>(_nh,
                                                                                            robot.handGuidingTopic);
    new(&splineMotionClient) actionlib::SimpleActionClient<iiwa_msgs::MoveAlongSplineAction>(_nh,
                                                                                             robot.moveAlongSplineTopic,
                                                                                             true);

    init(robot.armBaseFrame, robot.armToolFrame, robot.calibrationPinFrame, robot.pinLenght,
         robot.handGuidingTopic, robot.moveAlongSplineTopic, robot.transEeL);
    return *this;
}

unsigned int
Robot::init(const std::string &armBaseFrame, const std::string &armToolFrame,
            const std::string &calibrationPinFrame, const float &pinLenght, const std::string &handGuidingTopic,
            const std::string &moveAlongSplineTopic, const float &transEeL) {
    this->armBaseFrame = armBaseFrame;
    this->armToolFrame = armToolFrame;
    this->calibrationPinFrame = calibrationPinFrame;
    this->pinLenght = pinLenght;
    this->handGuidingTopic = handGuidingTopic;
    this->moveAlongSplineTopic = moveAlongSplineTopic;
    this->transEeL = transEeL;
    _id = robotId++;

    return _id;
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
    Eigen::Vector3d translation = Toolbox::toEigenVector(transformStamped.transform.translation);

    Eigen::Matrix3d rot = Toolbox::toEigenMatrix(transformStamped.transform.rotation);

    Eigen::Vector3d transPin(0, 0, pinLenght);

    translation += rot * transPin;

    return translation;

}

void Robot::gravityCompensation(bool activate) {
    _handGuidingClient.waitForServer();
    hand_guiding::HandGuidingGoal goal;
    goal.activateHandGuiding = activate;

    _handGuidingClient.sendGoal(goal);
    _handGuidingClient.waitForResult();

}

Eigen::Matrix4d Robot::getHomMatrixBaseTool() {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform(armBaseFrame, armToolFrame, ros::Time(0), ros::Duration(1));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("[ArmToLaserScannerCalibration] %s", ex.what());
        ros::Duration(1.0).sleep();
    }
    return Toolbox::toEigen(transformStamped);
}

void Robot::getHomMatrixLaserTool() {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform(armToolFrame, "hokuyo_laser", ros::Time(0), ros::Duration(1));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("[ArmToLaserScannerCalibration] %s", ex.what());
        ros::Duration(1.0).sleep();
    }
    Eigen::Matrix4d mat = Toolbox::toEigen(transformStamped);
    Toolbox::printMatrixToInfoStream(mat, " MAtrix Laser Tool:");
}

void Robot::moveToScanPosition(Eigen::Matrix4d h_B_O, float distanceY, float distanceZ, float offsetAlphaZ) {


    offsetAlphaZ = (offsetAlphaZ / 180.0) * M_PI;

    //LaserScannerFrame in EndeffectorFrame (approx.)
    Eigen::Vector3d xRot_EeL(0, 0, 1);
    Eigen::Vector3d yRot_EeL(1, 0, 0);
    Eigen::Vector3d zRot_EeL(0, 1, 0);
    Eigen::Vector3d trans_EeL(0, transEeL, 0);

    Eigen::Matrix4d h_Ee_L;
    h_Ee_L.setIdentity();
    h_Ee_L.block<3, 1>(0, 0) = xRot_EeL;
    h_Ee_L.block<3, 1>(0, 1) = yRot_EeL;
    h_Ee_L.block<3, 1>(0, 2) = zRot_EeL;
    h_Ee_L.block<3, 1>(0, 3) = trans_EeL;

    //EndeffectorFrame in LaserScannerFrame (approx.)
    Eigen::Matrix4d h_L_Ee;
    h_L_Ee.setIdentity();

    Eigen::Matrix3d rotH_L_Ee = h_Ee_L.block<3, 3>(0, 0).transpose();
    h_L_Ee.block<3, 3>(0, 0) = rotH_L_Ee;
    Eigen::Vector3d transH_L_Ee = (-rotH_L_Ee) * h_Ee_L.block<3, 1>(0, 3);
    h_L_Ee.block<3, 1>(0, 3) = transH_L_Ee;

    //LaserScannerPrimeFrame in ObjectFrame (approx.)
    Eigen::Vector3d xRot_OLprime(0, -1, 0);
    Eigen::Vector3d yRot_OLprime(-1, 0, 0);
    Eigen::Vector3d zRot_OLprime(0, 0, -1);
    Eigen::Vector3d trans_OLprime(0, distanceY, distanceZ);

    Eigen::Matrix4d h_O_Lprime;
    h_O_Lprime.setIdentity();
    h_O_Lprime.block<3, 1>(0, 0) = xRot_OLprime;
    h_O_Lprime.block<3, 1>(0, 1) = yRot_OLprime;
    h_O_Lprime.block<3, 1>(0, 2) = zRot_OLprime;
    h_O_Lprime.block<3, 1>(0, 3) = trans_OLprime;

    //LaserScannerFrame in LaserScannerPrimeFrame (specified rotation to improve measurement of w_meas)
    Eigen::Vector3d xRot_LprimeL(1, 0, 0);
    Eigen::Vector3d yRot_LprimeL(0, cos(offsetAlphaZ), sin(offsetAlphaZ));
    Eigen::Vector3d zRot_LprimeL(0, -sin(offsetAlphaZ), cos(offsetAlphaZ));
    Eigen::Vector3d trans_LprimeL(0, 0, 0);

    Eigen::Matrix4d h_Lprime_L;
    h_Lprime_L.setIdentity();
    h_Lprime_L.block<3, 1>(0, 0) = xRot_LprimeL;
    h_Lprime_L.block<3, 1>(0, 1) = yRot_LprimeL;
    h_Lprime_L.block<3, 1>(0, 2) = zRot_LprimeL;
    h_Lprime_L.block<3, 1>(0, 3) = trans_LprimeL;

    //LaserScannerFrame in ObjectFrame (approx.)
    Eigen::Matrix4d h_O_L;
    h_O_L = h_O_Lprime * h_Lprime_L;

    //EndeffectorFrame in BaseFrame (approx.)
    Eigen::Matrix4d h_B_Ee;
    h_B_Ee.setIdentity();

    h_B_Ee = h_B_O * h_O_L * h_L_Ee;
    Eigen::Matrix4d h_S = h_B_Ee;
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] Move robot arm to position: \n" << h_S);
    moveToPose(h_S);
}

void Robot::moveToHomePosition(float x, float y, float z) {
    Eigen::Matrix4d homTransMatrix;
    homTransMatrix.setIdentity();
    homTransMatrix(0, 3) = x;
    homTransMatrix(1, 3) = y;
    homTransMatrix(2, 3) = z;

    Eigen::Matrix3d rot;
    rot << -1, 0, 0,
            0, 1, 0,
            0, 0, -1;
    homTransMatrix.block<3, 3>(0, 0) = rot;

    moveToPose(homTransMatrix);
}

void Robot::moveToPose(Eigen::Matrix4d h_S) {
    iiwa_msgs::MoveAlongSplineGoal splineMotion;

    splineMotion.spline.segments.push_back(Toolbox::getSplineSegment(h_S, armBaseFrame, iiwa_msgs::SplineSegment::SPL));

    splineMotionClient.sendGoal(splineMotion);
    splineMotionClient.waitForResult();
}


bool Robot::setEndpointFrame(std::string frameId) {
    ROS_INFO_STREAM("Setting endpoint frame to \"" << frameId << "\"...");
    ros::ServiceClient setEndpointFrameClient = _nh.serviceClient<iiwa_msgs::SetEndpointFrame>(
            "/iiwa/configuration/setEndpointFrame");
    iiwa_msgs::SetEndpointFrame endpointFrame;
    endpointFrame.request.frame_id = frameId;
    if (!setEndpointFrameClient.call(endpointFrame)) {
        ROS_ERROR("Failed.");
        return false;
    } else if (!endpointFrame.response.success) {
        ROS_ERROR_STREAM("Service call returned error: " + endpointFrame.response.error);
        return false;
    }

    ROS_INFO("Done.");
    return true;
}


bool Robot::setPTPCartesianSpeedLimits(double maxCartesianVelocity) {
    ROS_INFO("Setting PTP Cartesian speed limits...");
    ros::ServiceClient setPTPCartesianSpeedLimitsClient = _nh.serviceClient<iiwa_msgs::SetPTPCartesianSpeedLimits>(
            "/iiwa/configuration/setPTPCartesianLimits");
    iiwa_msgs::SetPTPCartesianSpeedLimits cartesianSpeedLimits;
    cartesianSpeedLimits.request.maxCartesianVelocity = maxCartesianVelocity;
    cartesianSpeedLimits.request.maxCartesianAcceleration = 0.5;
    cartesianSpeedLimits.request.maxCartesianJerk = -1.0; // ignore
    cartesianSpeedLimits.request.maxOrientationVelocity = 0.5;
    cartesianSpeedLimits.request.maxOrientationAcceleration = 0.5;
    cartesianSpeedLimits.request.maxOrientationJerk = -1.0; // ignore
    if (!setPTPCartesianSpeedLimitsClient.call(cartesianSpeedLimits)) {
        ROS_ERROR("Failed.");
        return false;
    } else if (!cartesianSpeedLimits.response.success) {
        ROS_ERROR_STREAM("Service call returned error: " + cartesianSpeedLimits.response.error);
        return false;
    }

    ROS_INFO("Done.");
    return true;
}