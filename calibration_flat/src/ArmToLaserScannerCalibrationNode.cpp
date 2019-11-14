#include <calibration_flat/ArmToLaserScannerCalibrationNode.h>


ArmToLaserScannerCalibration::ArmToLaserScannerCalibration()
        : _actionServer(_nh, "arm_to_laser_scanner_calibration", false),
          _robot(_nh),
          _laserScanner(_nh),
          _userInput(_nh) {
    _actionServer.registerGoalCallback(boost::bind(&ArmToLaserScannerCalibration::executeCalibrationAction, this));

    importParameter();
    _actionServer.start();

    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] Action Server:                               " << _actionServer.isActive());

}

ArmToLaserScannerCalibration::~ArmToLaserScannerCalibration() {
}


void ArmToLaserScannerCalibration::importParameter() {
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] Namespace:                               " << _nh.getNamespace());

    armBaseFrame = "iiwa_link_0";
    armToolFrame = "iiwa_link_ee";
    calibrationPinFrame = "pin_link";
    calibrationObjectFrame = "calibration_object";
    laserScanTopic = "scan";
    handGuidingTopic = "/hand_guiding_node";
    std::string moveAlongSplineTopic = "/iiwa/action/move_along_spline";

    _nh.param<bool>("autostart", autostart, autostart);
    _nh.param<bool>("initial_calibration", initialCalibration, initialCalibration);
    _nh.param<float>("width_calib_obj", widthCalibrationObject, widthCalibrationObject);
    _nh.param<float>("angle_calib_obj", angleCalibrationObject, angleCalibrationObject);
    _nh.param<float>("near_distance", nearDistance, nearDistance);
    _nh.param<float>("far_distance", farDistance, farDistance);
    _nh.param<float>("offset_alpha_z", offsetAlphaZ, offsetAlphaZ);
    _nh.param<float>("offset_pre_scan", offsetPreScan, offsetPreScan);
    _nh.param<float>("calib_obj_offset_z", calibrationObjectOffsetZ, calibrationObjectOffsetZ);
    _nh.param<float>("trans_ee_l", transEeL, transEeL);
    _nh.param<float>("ransac_threshold", ransacThreshold, ransacThreshold);
    _nh.param<float>("tolerance_inlier", toleranceInlier, toleranceInlier);
    _nh.param<int>("number_of_scans", numberOfScans, numberOfScans);
    _nh.param<int>("number_of_iterations_ransac_ls", numberOfIterRansacLS, numberOfIterRansacLS);
    _nh.param<std::string>("arm_base_frame", armBaseFrame, armBaseFrame);
    _nh.param<std::string>("arm_tool_frame", armToolFrame, armToolFrame);
    _nh.param<std::string>("calibration_pin_frame", calibrationPinFrame, calibrationPinFrame);
    _nh.param<std::string>("laser_scan_topic", laserScanTopic, laserScanTopic);
//    _nh.param<std::string>("hand_guiding_topic", handGuidingTopic, handGuidingTopic);
    _nh.param<float>("delta_a_calib_obj", deltaACalibrationObject, deltaACalibrationObject);
    _nh.param<float>("home_pos_x", homePosX, homePosX);
    _nh.param<float>("home_pos_y", homePosY, homePosY);
    _nh.param<float>("home_pos_z", homePosZ, homePosZ);
    _nh.param<float>("pin_lenght", pinLenght, pinLenght);
    _nh.param<std::string>("target_directory", targetDir, targetDir);

    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] autostart:                               " << autostart);
    ROS_INFO_STREAM(
            "[ArmToLaserScannerCalibration] width calibration object:                        "
                    << widthCalibrationObject);
    ROS_INFO_STREAM(
            "[ArmToLaserScannerCalibration] angle calibration object:                " << angleCalibrationObject);
    ROS_INFO_STREAM(
            "[ArmToLaserScannerCalibration] delta a calibration object:              " << deltaACalibrationObject);
    ROS_INFO_STREAM(
            "[ArmToLaserScannerCalibration] offset_alpha_z                           " << offsetAlphaZ);
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] near distance:                           " << nearDistance);
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] far distance:                            " << farDistance);
    ROS_INFO_STREAM(
            "[ArmToLaserScannerCalibration] Home Position Robot:                     (" << homePosX << ", " << homePosY
                                                                                        << ", " << homePosZ << ")");
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] number of scans:                         " << numberOfScans);
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] Target Directory:                        " << targetDir);
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] Pin length:                              " << pinLenght);

    _robot = Robot(_nh, armBaseFrame, armToolFrame, calibrationPinFrame, pinLenght, handGuidingTopic,
                   moveAlongSplineTopic,transEeL);
    _laserScanner = LaserScanner(_nh, laserScanTopic, widthCalibrationObject, nearDistance, farDistance,
                                 numberOfIterRansacLS, targetDir,ransacThreshold, toleranceInlier);

}

void ArmToLaserScannerCalibration::spin() {
    if (autostart) {
        for (int countdown = 3; countdown > 0; countdown--) {
            ROS_INFO_STREAM("[ArmToLaserScannerCalibration] Starting calibration procedure in " << countdown);
            ros::Duration(1).sleep();
        }
        runCalibration();
    }
    ros::spin();
}

void ArmToLaserScannerCalibration::executeCalibrationAction(
        ) {
    calibration_flat::ArmToLaserScannerCalibrationGoalConstPtr goal = _actionServer.acceptNewGoal();
    initialCalibration = goal->initial_calibration;
    widthCalibrationObject = goal->width_calib_obj;
    angleCalibrationObject = goal->angle_calib_obj;
    deltaACalibrationObject = goal->delta_a_calib_obj;
    offsetAlphaZ = goal->offset_alpha_z;
    offsetPreScan = goal->offset_pre_scan;
    nearDistance = goal->near_distance;
    farDistance = goal->far_distance;
    calibrationObjectOffsetZ = goal->calib_obj_offset_z;
    transEeL = goal->trans_ee_l;
    pinLenght = goal->pin_lenght;
    numberOfScans = goal->number_of_scans;
    numberOfIterRansacLS = goal->number_of_iterations_ransac_ls;
    ransacThreshold = goal->ransac_threshold;
    toleranceInlier = goal->tolerance_inlier;
    homePosX = goal->home_pos_x;
    homePosY = goal->home_pos_y;
    homePosZ = goal->home_pos_z;
    targetDir = goal->target_directory;

    Eigen::Matrix4d h_T_L = runCalibration();

    calibration_flat::ArmToLaserScannerCalibrationResult result;
    Eigen::Quaterniond quaternion = Toolbox::toQuaternion(h_T_L);
    result.rot_x = quaternion.x();
    result.rot_y = quaternion.y();
    result.rot_z = quaternion.z();
    result.rot_w = quaternion.w();
    result.trans_x = h_T_L(0,3);
    result.trans_y = h_T_L(1,3);
    result.trans_z = h_T_L(2,3);

    _actionServer.setSucceeded(result);
}

Eigen::Matrix4d ArmToLaserScannerCalibration::runCalibration() {
    ROS_INFO_STREAM("runCalibration");
    _robot.setPTPCartesianSpeedLimits(0.25);
    _robot.setEndpointFrame(armToolFrame);

    Eigen::Matrix4d h_B_O;
    if (initialCalibration) {
        _robot.moveToHomePosition(homePosX, homePosY, homePosZ);
        ros::Duration(1).sleep();
        _robot.gravityCompensation(true);
        _userInput.getUserConfirmation("Please confirm the calibration point!");
        Eigen::Vector3d pinPosition1 = _robot.getPinPosition();
        _userInput.getUserConfirmation("Please confirm the calibration point!");
        Eigen::Vector3d pinPosition2 = _robot.getPinPosition();
        _userInput.getUserConfirmation("Please confirm the calibration point!");
        Eigen::Vector3d pinPosition3 = _robot.getPinPosition();
        _userInput.getUserConfirmation("Please move the calibration pin away from the calibration object!");
        _robot.gravityCompensation(false);
        _robot.moveToHomePosition(homePosX, homePosY, homePosZ);
        h_B_O = calculateHomMatrixBaseObject(pinPosition1, pinPosition2, pinPosition3);
        Toolbox::writeMatrixToFile(targetDir + "h_B_O", h_B_O);
        _userInput.getUserConfirmation("Please detach the calibration pin and attach the laser scanner");
    } else {
        h_B_O = Toolbox::readMatrixFromFile(targetDir + "h_B_O");
    }
    Toolbox::printMatrixToInfoStream(h_B_O, "Hom. Transformation Matrix h_B_O");

    //Home position
    _robot.moveToHomePosition(homePosX, homePosY, homePosZ);

    //Laser Scan 1
    _robot.moveToScanPosition(h_B_O, nearDistance + offsetPreScan, calibrationObjectOffsetZ, offsetAlphaZ);
    _robot.moveToScanPosition(h_B_O, nearDistance, calibrationObjectOffsetZ, offsetAlphaZ);
    std::vector<sensor_msgs::LaserScan> laserScans1 = _laserScanner.getLaserScan(numberOfScans);

    //Laser Scan 2
    _robot.moveToScanPosition(h_B_O, farDistance, calibrationObjectOffsetZ, offsetAlphaZ);
    std::vector<sensor_msgs::LaserScan> laserScans2 = _laserScanner.getLaserScan(numberOfScans);

    //Get h_B_T
    Eigen::Matrix4d h_B_T = _robot.getHomMatrixBaseTool();
    Toolbox::writeMatrixToFile(targetDir + "h_B_T", h_B_T);
    Toolbox::printMatrixToInfoStream(h_B_T, "Hom. Transformation Matrix h_B_T");

    //Back to Home Position
    _robot.moveToScanPosition(h_B_O, nearDistance + offsetPreScan, calibrationObjectOffsetZ, offsetAlphaZ);
    _robot.moveToHomePosition(homePosX, homePosY, homePosZ);

    //Laser Scan Data Processing
    sensor_msgs::LaserScan laserScan1 = _laserScanner.averageLaserScans(laserScans1);
    sensor_msgs::LaserScan laserScan2 = _laserScanner.averageLaserScans(laserScans2);
    CalibrationObjectPoints points = _laserScanner.extractAllCalibrationPoints(laserScan1, laserScan2);
    Toolbox::writeCalibrationPointsToFile(targetDir + "calibration_points", points);

    //Get h_L_O (Paper equations)
    Eigen::Matrix4d h_L_O = getHomMatrixLaserScannerObject(points);
    Toolbox::writeMatrixToFile(targetDir + "h_L_O", h_L_O);
    Toolbox::printMatrixToInfoStream(h_L_O, "Hom. Transformation Matrix h_L_O");

    //Get h_T_L (Laser Scanner frame in tool frame)
    Eigen::Matrix4d h_T_L = getHomMatrixToolLaserScanner(h_B_T, h_B_O, h_L_O);
    Toolbox::printMatrixToInfoStream(h_T_L, "Hom. Transformation Matrix h_T_L:");
    Toolbox::writeMatrixToFile(targetDir + "h_T_L", h_T_L);

    //Get Euler angles XYZ
    Eigen::Vector3d ea;
    Eigen::Matrix3d rot_t_L = h_T_L.block<3,3>(0,0);
    ea = rot_t_L.eulerAngles(2, 1, 0);
    Toolbox::printEulerZYXAndTranslation(ea, h_T_L);
    Toolbox::writeEulerZYXAndTranslationToFile(targetDir + "eulerAnglesXYZ", ea, h_T_L);
    return h_T_L;
}


Eigen::Matrix4d
ArmToLaserScannerCalibration::calculateHomMatrixBaseObject(Eigen::Vector3d position1, Eigen::Vector3d position2,
                                                           Eigen::Vector3d position3) {

    //Calculate Z and X unit vector for object frame from taken pin measurements
    Eigen::Vector3d z_target = position2 - position1;
    Eigen::Vector3d x_target = position3 - position1;
    z_target.normalize();
    x_target.normalize();
    Eigen::Vector3d z_unit = z_target;
    Eigen::Vector3d x_unit = x_target;

    //Calculate orthogonal Y unit vector with cross product
    //Ensure Y pointing upwards in space
    Eigen::Vector3d y_unit = z_unit.cross(x_unit);
    y_unit.normalize();

    Eigen::Vector3d orient_check(0, 0, 1);
    if (y_unit.dot(orient_check) < 0) {
        y_unit *= (-1);
    }

    //Renew X unit vector for full orthonormal system
    Eigen::Vector3d x_unit_new = y_unit.cross(z_unit);
    if (x_unit_new.dot(x_unit) < 0) {
        x_unit_new *= (-1);
    }
    x_unit_new.normalize();

    //Create transform h_B_O
    Eigen::Matrix4d trans;
    trans.setIdentity();
    trans.block<3, 1>(0, 0) = x_unit_new;
    trans.block<3, 1>(0, 1) = y_unit;
    trans.block<3, 1>(0, 2) = z_unit;
    trans.block<3, 1>(0, 3) = position1;

    return trans;
}

Eigen::Matrix4d ArmToLaserScannerCalibration::getHomMatrixLaserScannerObject(CalibrationObjectPoints points) {

    //Geometry of calibration object
    double w_meas_1;
    double leg_meas_1;
    double top_dist_1;
    double leg_meas_2;
    double top_dist_2;
    double rot_yl;
    double rot_xl;
    double rot_zl;
    double d_meas;

    Eigen::Vector3d temp1 = points.p1 - points.p2;
    w_meas_1 = (sqrt(temp1.dot(temp1)));
    if (w_meas_1 < 0){
        w_meas_1 = (-1.0);
    }

    //Rotation of object towards LaserScannerFrame adjusted negative (according to paper)
    rot_xl = -((M_PI / 2.0) - asin(widthCalibrationObject / (2.0 * w_meas_1)));
    if (offsetAlphaZ < 0) {
        rot_xl *= -1;
    }

    rot_zl = atan((points.p2[1] - points.p1[1]) / (points.p2[0] - points.p1[0])) + (M_PI / 2.0);
    if (rot_zl >= M_PI_2) {
        rot_zl = atan((points.p2[1] - points.p1[1]) / (points.p2[0] - points.p1[0])) - (M_PI / 2.0);
    }

    //Warning: w_meas can only be larger than widthCalibrationObject
    if (w_meas_1 * 2 <= widthCalibrationObject) {
        ROS_ERROR_STREAM("[ArmToLaserScannerCalibration] Error  w_meas_1:                    " << w_meas_1);
    }

    //Calculation first scan position (variable feature)
    Eigen::Vector3d temp2 = points.p2 - points.p3;
    leg_meas_1 = (sqrt(temp2.dot(temp2)));
    if (leg_meas_1<0){
        leg_meas_1 = (-1.0)*leg_meas_1;
    }
    top_dist_1 = sin(M_PI / 2.0 - rot_xl - angleCalibrationObject * M_PI / 180.0) * leg_meas_1 /
                 sin(angleCalibrationObject * M_PI / 180.0);
    if (rot_xl < 0) {
        top_dist_1 = sin(M_PI / 2.0 + rot_xl + angleCalibrationObject * M_PI / 180.0) * leg_meas_1 /
                     sin(angleCalibrationObject * M_PI / 180.0);
    }

    //Calculation second scan position (variable feature)
    Eigen::Vector3d temp4 = points.p5 - points.p6;
    leg_meas_2 = (sqrt(temp4.dot(temp4)));
    if (leg_meas_2<0){
        leg_meas_2 = (-1.0)*leg_meas_2;
    }
    top_dist_2 = sin(M_PI / 2.0 - rot_xl - angleCalibrationObject * M_PI / 180.0) * leg_meas_2 /
                 sin(angleCalibrationObject * M_PI / 180.0);
    if (rot_xl < 0) {
        top_dist_2 = sin(M_PI / 2.0 + rot_xl + angleCalibrationObject * M_PI / 180.0) * leg_meas_2 /
                     sin(angleCalibrationObject * M_PI / 180.0);
    }

    //Combining scans for rot_yl
    d_meas = (sqrt(pow(top_dist_2 - top_dist_1, 2.0) + pow(farDistance - nearDistance, 2.0)));
    if (d_meas < 0){
        d_meas = (-1.0)*d_meas;
    }
    rot_yl = -asin((top_dist_2 - top_dist_1) / d_meas);

    //Homogeneous transform object frame into laser scanner frame h_L_O:
    Eigen::Matrix4d trans_hlo;
    trans_hlo.setIdentity();
    trans_hlo(0, 0) = cos(rot_yl) * cos(rot_zl);
    trans_hlo(0, 1) = -cos(rot_yl) * sin(rot_zl);
    trans_hlo(0, 2) = sin(rot_yl);
    trans_hlo(0, 3) = points.p5[0];
    trans_hlo(1, 0) = (cos(rot_zl) * sin(rot_xl) * sin(rot_yl) + cos(rot_xl) * sin(rot_zl));
    trans_hlo(1, 1) = -sin(rot_xl) * sin(rot_yl) * sin(rot_zl) + cos(rot_xl) * cos(rot_zl);
    trans_hlo(1, 2) = -sin(rot_xl) * cos(rot_yl);
    trans_hlo(1, 3) = points.p5[1];
    trans_hlo(2, 0) = -cos(rot_xl) * cos(rot_zl) * sin(rot_yl) + sin(rot_xl) * sin(rot_zl);
    trans_hlo(2, 1) = cos(rot_xl) * sin(rot_yl) * sin(rot_zl) + cos(rot_zl) * sin(rot_xl);
    trans_hlo(2, 2) = cos(rot_xl) * cos(rot_yl);
    trans_hlo(2, 3) = (deltaACalibrationObject + top_dist_2);
    trans_hlo(3, 0) = 0;
    trans_hlo(3, 1) = 0;
    trans_hlo(3, 2) = 0;
    trans_hlo(3, 3) = 1;

    Eigen::Matrix3d rot_L_O;
    rot_L_O << 0, -1, 0,
            -1, 0, 0,
            0, 0, -1;

    trans_hlo.block<3,1>(0,3) = trans_hlo.block<3, 3>(0, 0) * trans_hlo.block<3,1>(0,3);
    trans_hlo.block<3, 3>(0, 0) = trans_hlo.block<3, 3>(0, 0) * rot_L_O;


    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] w_meas_1:                                " << w_meas_1);
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] rot_xl:                                  " << rot_xl << " rad \t," << (rot_xl* 180.0/M_PI) << " grad ");
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] rot_zl:                                  " << rot_zl << " rad \t," << (rot_zl* 180.0/M_PI) << " grad ");
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] leg_meas_1:                              " << leg_meas_1);
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] top_dist_1:                              " << top_dist_1);
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] leg_meas_2:                              " << leg_meas_2);
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] top_dist_2:                              " << top_dist_2);
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] d_meas:                                  " << d_meas);
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] rot_yl:                                  " << rot_yl << " rad \t," << (rot_yl* 180.0/M_PI) << " grad ");
    return trans_hlo;
}

Eigen::Matrix4d ArmToLaserScannerCalibration::getHomMatrixToolLaserScanner(Eigen::Matrix4d h_B_T, Eigen::Matrix4d h_B_O,
                                                                           Eigen::Matrix4d h_L_O) {
    Eigen::Matrix4d h_T_B;
    h_T_B.setIdentity();
    Eigen::Matrix3d rotH_T_B = h_B_T.block<3, 3>(0, 0).transpose();
    h_T_B.block<3, 3>(0, 0) = rotH_T_B;
    Eigen::Vector3d transH_T_B = (-rotH_T_B) * h_B_T.block<3, 1>(0, 3);
    h_T_B.block<3, 1>(0, 3) = transH_T_B;
    Toolbox::printMatrixToInfoStream(h_T_B, "Hom. Transformation Matrix h_T_B");

    Eigen::Matrix4d h_O_L;
    h_O_L.setIdentity();
    Eigen::Matrix3d rotH_O_L = h_L_O.block<3, 3>(0, 0).transpose();
    h_O_L.block<3, 3>(0, 0) = rotH_O_L;
    Eigen::Vector3d transH_O_L = (-rotH_O_L) * h_L_O.block<3, 1>(0, 3);
    h_O_L.block<3, 1>(0, 3) = transH_O_L;

    Toolbox::printMatrixToInfoStream(h_O_L, "Hom. Transformation Matrix h_O_L");

    Eigen::Matrix4d h_T_L = h_T_B * h_B_O * h_O_L;

    return h_T_L;
}