#ifndef LASER_SCANNER_H
#define LASER_SCANNER_H


#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <calibration_flat/CalibrationObjectPoints.h>
#include <math.h>
#include <calibration_flat/Toolbox.h>
#include <random>
#include <limits>


class LaserScanner {
public:
    LaserScanner(ros::NodeHandle &nh);

    LaserScanner(ros::NodeHandle &nh, std::string laserScanTopic, const float widthCalibrationObject,
                 const float nearDistance, const float farDistance, const int numberOfIterRansacLS,
                 const std::string &targetDir, const float &ransacThreshold, const float &toleranceInlier);

    virtual ~LaserScanner();

    LaserScanner &operator=(const LaserScanner &laserScanner);

    std::vector<sensor_msgs::LaserScan> getLaserScan(uint16_t numberOfScans);

    void laserScannerCallback(const sensor_msgs::LaserScan &msg);

    sensor_msgs::LaserScan averageLaserScans(std::vector<sensor_msgs::LaserScan> vector);

    CalibrationObjectPoints extractAllCalibrationPoints(sensor_msgs::LaserScan scan1, sensor_msgs::LaserScan scan2);

private:
    ros::NodeHandle &_nh;
    bool saveLaserScan;
    std::vector<sensor_msgs::LaserScan> laserScanVector;
    std::string laserScanTopic;
    float widthCalibrationObject;
    float nearDistance;
    float farDistance;
    float toleranceInlier;
    float ransacThreshold;
    std::string targetDir;
    int numberOfIterRansacLS;
    unsigned int _id;
    ros::Subscriber laserScanSub;


    unsigned int init(const std::string &laserScanTopic, const float &widthCalibrationObject, const float &nearDistance,
                      const float &farDistance, const int &numberOfIterRansacLS, const std::string &targetDir,  const float &ransacThreshold, const float &toleranceInlier);

    void extractPoints(std::vector<float> ranges, float angleIncrement, float angleMin, int scan,
                       CalibrationObjectPoints &pPoints, double distance, float maxDepth, double threshold,
                       double tolerance);

    bool
    testNextPointsOutsideTolerance(std::vector<double> &deltas, int index, int nextIndexes, bool inside,
                                   double percent, double tolerance);

    void ransacLaserScanData(float maxDepth, const std::vector<Eigen::Vector2d> &xyLaserScanDataPoints,
                             double threshold, Eigen::Vector2d &a_model,
                             Eigen::Vector2d &b_model, Eigen::Vector2d &line_vec_model,
                             Eigen::Vector2d &normal_model) const;

    std::vector<Eigen::Vector2d>
    getOnlyInliers(const std::vector<Eigen::Vector2d> &xyLaserScanDataPoints, double threshold,
                   Eigen::Vector2d &a_model, const Eigen::Vector2d &normal_model,
                   const Eigen::Vector2d &line_vec_model) const;

    void leastSquareLaserScanData(Eigen::Vector2d &line_vec_model,
                                  const std::vector<Eigen::Vector2d> &xyLaserScanDataPointReduced,
                                  Eigen::Vector2d &a_model, Eigen::Vector2d &b_model,
                                  Eigen::Vector2d &normal_model) const;

    std::vector<double>
    reduceLaserScanDataByAngle(const std::vector<float> &ranges, float angleIncrement, float angleMin,
                               double distance) const;

    std::vector<Eigen::Vector2d>
    reduceLaserScanDataByMaxDepth(float maxDepth, const std::vector<Eigen::Vector2d> &xyLaserScanDataPoints,
                                  int scan) const;

    std::vector<double>
    calcDeltasForModel(int scan, const std::vector<Eigen::Vector2d> &xyLaserScanDataPoints, Eigen::Vector2d &a_model,
                       const Eigen::Vector2d &line_vec_model) const;

    void extractPointsFromLine(int scan, CalibrationObjectPoints &pPoints, double tolerance,
                               const std::vector<Eigen::Vector2d> &xyLaserScanDataPoints, Eigen::Vector2d &a_model,
                               const Eigen::Vector2d &line_vec_model, std::vector<double> &xyDeltaToModel);
};

#endif
