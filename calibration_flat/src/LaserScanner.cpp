#include <calibration_flat/LaserScanner.h>

unsigned int laserScannerId = 0;

LaserScanner::LaserScanner(ros::NodeHandle &nh) :
        _nh(nh) {


}

LaserScanner::~LaserScanner() {
}

LaserScanner::LaserScanner(ros::NodeHandle &nh, std::string laserScanTopic, const float widthCalibrationObject,
                           const float nearDistance, const float farDistance, const int numberOfIterRansacLS,
                           const std::string &targetDir,  const float &ransacThreshold, const float &toleranceInlier) : _nh(nh) {
    init(laserScanTopic, widthCalibrationObject, nearDistance, farDistance, numberOfIterRansacLS, targetDir,ransacThreshold,toleranceInlier);
}

LaserScanner &LaserScanner::LaserScanner::operator=(const LaserScanner &laserScanner) {
    if (&laserScanner == this) {
        return *this;
    }
    _nh = laserScanner._nh;

    init(laserScanner.laserScanTopic, laserScanner.widthCalibrationObject, laserScanner.nearDistance,
         laserScanner.farDistance, laserScanner.numberOfIterRansacLS, laserScanner.targetDir, laserScanner.ransacThreshold, laserScanner.toleranceInlier);
    return *this;
}

unsigned int
LaserScanner::init(const std::string &laserScanTopic, const float &widthCalibrationObject, const float &nearDistance,
                   const float &farDistance, const int &numberOfIterRansacLS, const std::string &targetDir,  const float &ransacThreshold, const float &toleranceInlier) {
    this->laserScanTopic = laserScanTopic;
    this->widthCalibrationObject = widthCalibrationObject;
    this->farDistance = farDistance;
    this->nearDistance = nearDistance;
    this->numberOfIterRansacLS = numberOfIterRansacLS;
    this->targetDir = targetDir;
    this->ransacThreshold = ransacThreshold;
    this->toleranceInlier = toleranceInlier;
    laserScanSub = _nh.subscribe(laserScanTopic, 1, &LaserScanner::laserScannerCallback, this);
    _id = laserScannerId++;

    return _id;
}


std::vector<sensor_msgs::LaserScan> LaserScanner::getLaserScan(uint16_t numberOfScans) {
    ros::spinOnce();
    laserScanVector.clear();
    if (!laserScanVector.empty()) {
        ROS_ERROR_STREAM(
                "[ArmToLaserScannerCalibration] Laser Scan not empty:                   " << laserScanVector.size());
    }
    saveLaserScan = true;
    uint32_t startTime = time(NULL);
    while (laserScanVector.size() <= numberOfScans && difftime(time(NULL), startTime) < 60) {
        ros::spinOnce();
    }
    saveLaserScan = false;
    laserScanVector.resize(numberOfScans);
    ROS_INFO_STREAM(
            "[ArmToLaserScannerCalibration] Number of scans taken:                   " << laserScanVector.size());
    return laserScanVector;
}

void LaserScanner::laserScannerCallback(const sensor_msgs::LaserScan &msg) {
    if (saveLaserScan) {
        laserScanVector.push_back(msg);
    }
}

sensor_msgs::LaserScan LaserScanner::averageLaserScans(std::vector<sensor_msgs::LaserScan> vector) {

    sensor_msgs::LaserScan averageLaserScan;
    if (vector.empty() && vector.size() < 1) {
        ROS_ERROR_STREAM("[ArmToLaserScannerCalibration] Error: no laser scan data");
        return averageLaserScan;
    }
    averageLaserScan.angle_increment = vector.front().angle_increment;
    averageLaserScan.angle_min = vector.front().angle_min;
    averageLaserScan.angle_max = vector.front().angle_max;

    unsigned int sizeRanges = vector.front().ranges.size();
    std::vector<double> temp1;
    temp1.resize(sizeRanges, 0);

    for (std::vector<sensor_msgs::LaserScan>::iterator it = vector.begin(); it != vector.end(); it++) {
        for (int i = 0; i < it->ranges.size(); i++) {
            temp1[i] += it->ranges[i];
        }
    }

    unsigned int anzLaserScans = vector.size();
    std::vector<float> temp2;
    temp2.resize(sizeRanges, 0);

    for (int i = 0; i < sizeRanges; i++) {
        double averPoint = temp1[i] / anzLaserScans;
        if (averPoint >= 0 && averPoint < 500000) {
            temp2[i] = averPoint;
        } else {
            temp2[i] = 50;
        }
    }

    averageLaserScan.ranges = temp2;


    return averageLaserScan;
}


CalibrationObjectPoints LaserScanner::extractAllCalibrationPoints(sensor_msgs::LaserScan scan1,
                                                                  sensor_msgs::LaserScan scan2) {
    CalibrationObjectPoints points;
    //Threshold 0.15 enforces, that no points of background are extracted by accident
    extractPoints(scan1.ranges, scan1.angle_increment, scan1.angle_min, 1, points, nearDistance,
                  nearDistance + 0.15, ransacThreshold, toleranceInlier);
    extractPoints(scan2.ranges, scan2.angle_increment, scan2.angle_min, 2, points, farDistance,
                  farDistance + 0.15, ransacThreshold, toleranceInlier);

    return points;
}

void
LaserScanner::extractPoints(std::vector<float> ranges, float angleIncrement, float angleMin, int scan,
                            CalibrationObjectPoints &pPoints, double distance, float maxDepth, double threshold,
                            double tolerance) {

    //Limitation of observed angle area
    std::vector<double> reducedRanges = reduceLaserScanDataByAngle(ranges, angleIncrement, angleMin, distance);


    //Transformation of measured ranges into xy-coordinates
    std::vector<Eigen::Vector2d> xyLaserScanDataPoints;

    for (int i = 0; i < reducedRanges.size(); i++) {
        double winkel = (i - round(reducedRanges.size() / 2)) * angleIncrement;
        Eigen::Vector2d vector2D(cos(winkel) * reducedRanges[i], sin(winkel) * reducedRanges[i]);
        xyLaserScanDataPoints.push_back(vector2D);
    }

    Toolbox::writeVectorToFile(targetDir + "/reducedranges" + std::to_string(scan), xyLaserScanDataPoints);

    //Depth limitation
    std::vector<Eigen::Vector2d> rangesForeground = reduceLaserScanDataByMaxDepth(maxDepth, xyLaserScanDataPoints,
                                                                                  scan);

    //Define model parameters: Line equation in vector form
    Eigen::Vector2d a_model;
    Eigen::Vector2d b_model;
    Eigen::Vector2d line_vec_model;
    Eigen::Vector2d normal_model;

    //Line Fitting with RANSAC & LS

    ransacLaserScanData(maxDepth, rangesForeground, threshold, a_model, b_model,
                        line_vec_model,
                        normal_model);
    std::vector<Eigen::Vector2d> xyLaserScanDataPointReduced = getOnlyInliers(rangesForeground, threshold, a_model,
                                                                              normal_model, line_vec_model);
    Toolbox::writeVectorToFile(targetDir + "/inlier" + std::to_string(scan), xyLaserScanDataPointReduced);

    leastSquareLaserScanData(line_vec_model, xyLaserScanDataPointReduced, a_model,
                             b_model, normal_model);

    //Check the single vectors on divergence towards the found line
    std::vector<double> xyDeltaToModel = calcDeltasForModel(scan, xyLaserScanDataPoints, a_model, line_vec_model);

    //Extract points from line
    extractPointsFromLine(scan, pPoints, tolerance, xyLaserScanDataPoints, a_model, line_vec_model, xyDeltaToModel);


}

void LaserScanner::extractPointsFromLine(int scan, CalibrationObjectPoints &pPoints, double tolerance,
                                         const std::vector<Eigen::Vector2d> &xyLaserScanDataPoints,
                                         Eigen::Vector2d &a_model, const Eigen::Vector2d &line_vec_model,
                                         std::vector<double> &xyDeltaToModel) {
    //Find begin and endpoints of line by checking behavior of neighboring points
    bool leftPoint = true;
    int indexLeftPoint = 0;
    bool middlePoint = false;
    int indexMiddlePoint = 0;
    bool rightPoint = false;
    int indexRightPoint = 0;


    for (int i = 0; i < xyDeltaToModel.size(); i++) {
        if (xyDeltaToModel[i] > tolerance) {
            continue;
        }
        if (testNextPointsOutsideTolerance(xyDeltaToModel, i, 10, true, 0.9, tolerance) && leftPoint) {
            indexLeftPoint = i;
            leftPoint = false;
            middlePoint = true;
        } else if (testNextPointsOutsideTolerance(xyDeltaToModel, i, 10, false, 0.9, tolerance) && middlePoint) {
            indexMiddlePoint = i;
            middlePoint = false;
            rightPoint = true;
        } else if (testNextPointsOutsideTolerance(xyDeltaToModel, i, 10, true, 0.9, tolerance) && rightPoint) {
            indexRightPoint = i;
            rightPoint = false;
        }
    }
    if (leftPoint || middlePoint || rightPoint) {
        ROS_ERROR_STREAM("[ArmToLaserScannerCalibration] Error: Not all points could be determined");
    }

    Eigen::Vector2d tempP_a = xyLaserScanDataPoints[indexLeftPoint] +
                              ((a_model - xyLaserScanDataPoints[indexLeftPoint]) -
                               ((a_model - xyLaserScanDataPoints[indexLeftPoint]).dot(line_vec_model)) *
                               line_vec_model);
    Eigen::Vector2d tempP_b = xyLaserScanDataPoints[indexMiddlePoint] +
                              ((a_model - xyLaserScanDataPoints[indexMiddlePoint]) -
                               ((a_model - xyLaserScanDataPoints[indexMiddlePoint]).dot(line_vec_model)) *
                               line_vec_model);
    Eigen::Vector2d tempP_c = xyLaserScanDataPoints[indexRightPoint] +
                              ((a_model - xyLaserScanDataPoints[indexRightPoint]) -
                               ((a_model - xyLaserScanDataPoints[indexRightPoint]).dot(line_vec_model)) *
                               line_vec_model);

    Eigen::Vector3d p_a(tempP_a.x(), tempP_a.y(), 0);
    Eigen::Vector3d p_b(tempP_b.x(), tempP_b.y(), 0);
    Eigen::Vector3d p_c(tempP_c.x(), tempP_c.y(), 0);

    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] p_a: \n" << p_a);
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] p_b: \n" << p_b);
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] p_c: \n" << p_c);

    if (scan == 1) {
        pPoints.p1 = p_a;
        pPoints.p2 = p_b;
        pPoints.p3 = p_c;
    } else if (scan == 2) {
        pPoints.p4 = p_a;
        pPoints.p5 = p_b;
        pPoints.p6 = p_c;
    } else {
        ROS_ERROR_STREAM("[ArmToLaserScannerCalibration] Error");
    }
}

std::vector<double>
LaserScanner::calcDeltasForModel(int scan, const std::vector<Eigen::Vector2d> &xyLaserScanDataPoints,
                                 Eigen::Vector2d &a_model, const Eigen::Vector2d &line_vec_model) const {
    std::vector<double> xyDeltaToModel;
    for (int i = 0; i < xyLaserScanDataPoints.size(); i++) {
        double deltaToModel;
        Eigen::Vector2d tem1 = -(a_model - xyLaserScanDataPoints[i]);
        Eigen::Vector2d tem2 = tem1 - tem1.dot(line_vec_model) * line_vec_model;
        deltaToModel = sqrt(tem2.dot(tem2));
        if (deltaToModel < 0){
            deltaToModel = (-1.0)*deltaToModel;
        }
        xyDeltaToModel.push_back(deltaToModel);
    }
    return xyDeltaToModel;
}

std::vector<Eigen::Vector2d>
LaserScanner::reduceLaserScanDataByMaxDepth(float maxDepth, const std::vector<Eigen::Vector2d> &xyLaserScanDataPoints,
                                            int scan) const {
    std::vector<Eigen::Vector2d> rangesForeground;

    for (int i = 0; i < xyLaserScanDataPoints.size(); i++) {
        if (xyLaserScanDataPoints[i].x() < maxDepth) {
            rangesForeground.push_back(xyLaserScanDataPoints[i]);
        }
    }
    return rangesForeground;
}

std::vector<double>
LaserScanner::reduceLaserScanDataByAngle(const std::vector<float> &ranges, float angleIncrement, float angleMin,
                                         double distance) const {
    double theta_required = (atan(widthCalibrationObject / (2 * distance)) + (5.0 / 180.0) * M_PI);

    ROS_INFO_STREAM(
            "[ArmToLaserScannerCalibration] theta_required:                   " << ((theta_required * 180) / M_PI));

    int middle = (angleMin / angleIncrement);
    if (middle<0){
        middle = (-1)*middle;
    }
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] middle: \n" << middle);
    int jCounter = round(theta_required / angleIncrement);

    std::vector<double> reducedRanges;
    reducedRanges.resize(jCounter * 2 + 1, 0);
    int reducedRangesIndex = 0;

    for (int i = 0; i < ranges.size(); i++) {
        if (i <= (middle + jCounter) && i >= (middle - jCounter)) {
            reducedRanges[reducedRangesIndex] = ranges[i];
            reducedRangesIndex++;
        }
    }
    return reducedRanges;
}

void LaserScanner::leastSquareLaserScanData(Eigen::Vector2d &line_vec_model,
                                            const std::vector<Eigen::Vector2d> &xyLaserScanDataPointReduced,
                                            Eigen::Vector2d &a_model, Eigen::Vector2d &b_model,
                                            Eigen::Vector2d &normal_model) const {
    double bestSquaredDistance = std::numeric_limits<double>::max();
    for (int iter = 0; iter < numberOfIterRansacLS; iter++) {
        std::random_device random_device;
        std::mt19937 engine{random_device()};
        std::uniform_int_distribution<int> dist(0, xyLaserScanDataPointReduced.size() - 1);

        int intRandom1, intRandom2;

        do {
            intRandom1 = dist(engine);
            intRandom2 = dist(engine);
        } while (intRandom1 == intRandom2);

        Eigen::Vector2d a_line = xyLaserScanDataPointReduced[intRandom1];
        Eigen::Vector2d b_line = xyLaserScanDataPointReduced[intRandom2];

        Eigen::Vector2d differ_ab = b_line - a_line;
        differ_ab.normalize();

        Eigen::Vector2d normal_vec(-differ_ab[1], differ_ab[0]);

        double squaredDistance = 0;

        for (int i = 0; i < xyLaserScanDataPointReduced.size(); i++) {
            double orthDist;
            Eigen::Vector2d tem1 = (a_line - xyLaserScanDataPointReduced[i]);
            Eigen::Vector2d tem2 = tem1 - tem1.dot(differ_ab) * differ_ab;
            orthDist = sqrt(tem2.dot(tem2));
            if (orthDist<0){
                orthDist = (-1.0)*orthDist;
            }
            squaredDistance += pow(orthDist * 100, 2);
        }

        if (squaredDistance < bestSquaredDistance) {
            bestSquaredDistance = squaredDistance;
            a_model = a_line;
            b_model = b_line;
            line_vec_model = differ_ab;
            normal_model = normal_vec;

            ROS_INFO_STREAM("[ArmToLaserScannerCalibration] a_model: \n" << a_model);
            ROS_INFO_STREAM("[ArmToLaserScannerCalibration] b_model: \n" << b_model);
        }
    }
}

std::vector<Eigen::Vector2d>
LaserScanner::getOnlyInliers(const std::vector<Eigen::Vector2d> &xyLaserScanDataPoints, double threshold,
                             Eigen::Vector2d &a_model, const Eigen::Vector2d &normal_model,
                             const Eigen::Vector2d &line_vec_model) const {
    std::vector<Eigen::Vector2d> xyLaserScanDataPointReduced;
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] length_before: \n" << xyLaserScanDataPoints.size());
    for (int i = 0; i < xyLaserScanDataPoints.size(); i++) {
        double orthDist;
        Eigen::Vector2d tem1 = -(a_model - xyLaserScanDataPoints[i]);
        Eigen::Vector2d tem2 = tem1 - tem1.dot(line_vec_model) * line_vec_model;
        orthDist = sqrt(tem2.dot(tem2));
        if (orthDist < 0.0){
            orthDist = (-1.0)*orthDist;
        }
        if (orthDist < threshold) {
            if (orthDist != 0.0){
            }
            xyLaserScanDataPointReduced.push_back(xyLaserScanDataPoints[i]);
        }
    }
    ROS_INFO_STREAM("[ArmToLaserScannerCalibration] length_after: \n" << xyLaserScanDataPointReduced.size());
    return xyLaserScanDataPointReduced;
}

void LaserScanner::ransacLaserScanData(float maxDepth, const std::vector<Eigen::Vector2d> &xyLaserScanDataPoints,
                                       double threshold, Eigen::Vector2d &a_model,
                                       Eigen::Vector2d &b_model, Eigen::Vector2d &line_vec_model,
                                       Eigen::Vector2d &normal_model) const {
    int bestNumberOfInlier = 0;
    for (int iter = 0; iter < numberOfIterRansacLS; iter++) {

        std::random_device random_device;
        std::mt19937 engine{random_device()};
        std::uniform_int_distribution<int> dist(0, xyLaserScanDataPoints.size() - 1);

        int intRandom1, intRandom2;

        do {
            intRandom1 = dist(engine);
            intRandom2 = dist(engine);
        } while (intRandom1 == intRandom2);


        Eigen::Vector2d a_line = xyLaserScanDataPoints[intRandom1]; //dist(engine)
        Eigen::Vector2d b_line = xyLaserScanDataPoints[intRandom2];

        if (a_line.x() > maxDepth || b_line.x() > maxDepth) {
            continue;
        }

        Eigen::Vector2d differ_ab = b_line - a_line;
        differ_ab.normalize();

        Eigen::Vector2d normal_vec(-differ_ab[1], differ_ab[0]);

        int numberOfInlier = 0;

        for (int i = 0; i < xyLaserScanDataPoints.size(); i++) {
            double orthDist;
            Eigen::Vector2d tem1 = (a_line - xyLaserScanDataPoints[i]);
            Eigen::Vector2d tem2 = tem1 - tem1.dot(differ_ab) * differ_ab;
            orthDist = sqrt(tem2.dot(tem2));
            if (orthDist < 0){
                orthDist = (-1.0)*orthDist;
            }
            if (orthDist < threshold) {
                numberOfInlier += 1;
            }
        }

        if (numberOfInlier > bestNumberOfInlier) {
            bestNumberOfInlier = numberOfInlier;
            a_model = a_line;
            b_model = b_line;
            line_vec_model = differ_ab;
            normal_model = normal_vec;
        }
    }
}

bool LaserScanner::testNextPointsOutsideTolerance(std::vector<double> &deltas, int index, int nextIndexes, bool inside,
                                                  double percent, double tolerance) {
    double numberOfInside = 0;
    double numberOfOutside = 0;
    for (int i = index + 1; i < deltas.size() && i < index + nextIndexes + 1; i++) {
        if (deltas[i] < tolerance) {
            numberOfInside += 1;
        } else {
            numberOfOutside += 1;
        }
    }
    double total = numberOfInside + numberOfOutside;

    if (inside) {
        if ((numberOfInside / total) >= percent) {
            return true;
        }
    } else {
        if ((numberOfOutside / total) >= percent) {
            return true;
        }
    }
    return false;
}