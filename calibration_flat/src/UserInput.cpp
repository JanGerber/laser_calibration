#include <calibration_flat/UserInput.h>


UserInput::UserInput(ros::NodeHandle &nh) :
        _nh(nh) {
    subscriberUserConfirmation = _nh.subscribe("user_confirmation", 1, &UserInput::userConfirmationCallback, this);
}

UserInput::~UserInput() {

}

void UserInput::getUserConfirmation(std::string message) {

    userConfirmation = false;
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] " << message << " (Topic: /user_confirmation)");
    while (userConfirmation == false) {
        ros::spinOnce();
        if (userConfirmation) {
            return;
        }
    }
}

void UserInput::userConfirmationCallback(const calibration_flat::UserConfirmation &msg) {
    userConfirmation = msg.userConfirmation;
}








