#ifndef EXTRACTEDPOINTS_H
#define EXTRACTEDPOINTS_H

#include <Eigen/Dense>

class CalibrationObjectPoints {
public:
    CalibrationObjectPoints();

    ~CalibrationObjectPoints();

    Eigen::Vector3d p1;
    Eigen::Vector3d p2;
    Eigen::Vector3d p3;
    Eigen::Vector3d p4;
    Eigen::Vector3d p5;
    Eigen::Vector3d p6;
};


#endif //EXTRACTEDPOINTS_H
