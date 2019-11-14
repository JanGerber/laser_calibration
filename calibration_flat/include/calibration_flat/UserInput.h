#ifndef USERINPUT_H
#define USERINPUT_H

#include <ros/ros.h>
#include <calibration_flat/UserConfirmation.h>



class UserInput {
public:
    UserInput(ros::NodeHandle &nh);
    virtual ~UserInput() ;

    void getUserConfirmation(std::string message);
    void userConfirmationCallback(const calibration_flat::UserConfirmation& msg) ;
private:
    ros::NodeHandle &_nh;
    ros::Subscriber subscriberUserConfirmation;

    bool userConfirmation;
};


#endif //USERINPUT_H
