#ifndef TOOLBOX_H
#define TOOLBOX_H

#include <ros/ros.h>
#include <fstream>
#include <string.h>
#include <eigen3/Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <math.h>
#include <calibration_flat/CalibrationObjectPoints.h>
#include <iiwa_msgs/SplineSegment.h>

class Toolbox {
public:
    static void writeMatrixToFile(const std::string &fileName, Eigen::Matrix4d &matrix) {
        const Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ");
        std::string nameAndTyp = fileName + ".csv";
        try {
            std::ofstream file(nameAndTyp);
            file << matrix.format(CSVFormat);
        } catch (const std::ofstream::failure &err) {
            ROS_ERROR_STREAM("[ArmToLaserScannerCalibration] writeMatrixToFile Error: " << err.what());
        }

    }

    static void writeVectorToFile(const std::string &fileName, std::vector<Eigen::Vector2d> vector) {
        std::string nameAndTyp = fileName + ".csv";
        try {
            std::ofstream file(nameAndTyp);
            for (const auto &e : vector) file << e.x() << ", " << e.y() << "\n";
        } catch (const std::ofstream::failure &err) {
            ROS_ERROR_STREAM("[ArmToLaserScannerCalibration] writeMatrixToFile Error: " << err.what());
        }

    }
    static void writeVectorToFile(const std::string &fileName, Eigen::Vector3d vector) {
        const Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ");
        std::string nameAndTyp = fileName + ".csv";
        try {
            std::ofstream file(nameAndTyp);
            file << vector.format(CSVFormat);
        } catch (const std::ofstream::failure &err) {
            ROS_ERROR_STREAM("[ArmToLaserScannerCalibration] writeVectorToFile Error: " << err.what());
        }

    }
    static void writeVectorToFile(const std::string &fileName, std::vector<double> vector) {
        std::string nameAndTyp = fileName + ".csv";
        try {
            std::ofstream file(nameAndTyp);
            for (const auto &e : vector) file << e << "\n";
        } catch (const std::ofstream::failure &err) {
            ROS_ERROR_STREAM("[ArmToLaserScannerCalibration] writeMatrixToFile Error: " << err.what());
        }

    }

    static Eigen::Matrix4d readMatrixFromFile(const std::string &fileName) {
        std::ifstream indata;
        std::string nameAndTyp = fileName + ".csv";
        indata.open(nameAndTyp);
        std::string line;
        std::vector<double> values;
        uint rows = 0;
        while (std::getline(indata, line)) {
            std::stringstream lineStream(line);
            std::string cell;
            char delimiter = ',';
            while (std::getline(lineStream, cell, delimiter)) {
                values.push_back(std::stod(cell));
            }
            rows++;
        }
        Eigen::Matrix4d matrix(values.data());
        matrix.transposeInPlace();
        return matrix;

    }

    static Eigen::Quaterniond toEigenQuaternion(geometry_msgs::Quaternion quaternion) {
        Eigen::Quaterniond quaternionEigen( quaternion.w,quaternion.x, quaternion.y, quaternion.z);
        return quaternionEigen;
    }

    static Eigen::Matrix4d toEigen(const geometry_msgs::TransformStamped &tf) {
        Eigen::Quaterniond rotation = toEigenQuaternion(tf.transform.rotation);
        Eigen::Vector3d translation = toEigenVector(tf.transform.translation);

        Eigen::Matrix4d result;
        result.setIdentity();
        result.block<3, 3>(0, 0) = rotation.toRotationMatrix();
        result.block<3, 1>(0, 3) = translation;

        return result;
    }

    static void printMatrixToInfoStream(Eigen::Matrix4d matrix, std::string matrixName) {
        ROS_INFO_STREAM("[ArmToLaserScannerCalibration]" << matrixName << "\n" << matrix);
    }

    static void printVectorToInfoStream(Eigen::Vector3d vector) {
        ROS_INFO_STREAM("\n" << vector);
    }

    static Eigen::Vector3d toEigenVector(const tf2::Vector3 vector) {
        Eigen::Vector3d eigenVec;
        eigenVec << vector[0], vector[1], vector[2];
        return eigenVec;
    }

    static Eigen::Vector3d toEigenVector(const geometry_msgs::Vector3 vector) {
        Eigen::Vector3d eigenVec;
        eigenVec << vector.x, vector.y, vector.z;
        return eigenVec;
    }

    static inline geometry_msgs::Pose toPoseMsg(const Eigen::Matrix4d h_S) {
        geometry_msgs::Pose msg;

        msg.position.x = h_S(0, 3);
        msg.position.y = h_S(1, 3);
        msg.position.z = h_S(2, 3);
        Eigen::Quaterniond q = toQuaternion(h_S);
        msg.orientation.w = q.w();
        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        return msg;
    }

    static inline Eigen::Quaterniond toQuaternion(const Eigen::Matrix4d &matrix) {
        Eigen::Matrix3d rot;
        rot << matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(1, 0), matrix(1, 1), matrix(1, 2), matrix(2, 0), matrix(
                2, 1), matrix(2, 2);
        Eigen::Quaterniond quaternion(rot);
        return quaternion;
    }

    static Eigen::Matrix3d toEigenMatrix(geometry_msgs::Quaternion quaternion) {
        Eigen::Quaterniond eigenQuaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        return eigenQuaternion.toRotationMatrix();
    }

    static void writeCalibrationPointsToFile(std::string filename, CalibrationObjectPoints &points) {
        std::string nameAndTyp = filename + ".csv";
        try {
            std::ofstream file(nameAndTyp);
           file << points.p1.x() << ", " << points.p1.y() << ", "  << points.p1.z()  << "\n";
           file << points.p2.x() << ", " << points.p2.y() << ", " << points.p2.z()  << "\n";
           file << points.p3.x() << ", " << points.p3.y() << ", " << points.p3.z()  << "\n";
           file << points.p4.x() << ", " << points.p4.y() << ", " << points.p4.z()  << "\n";
           file << points.p5.x() << ", " << points.p5.y() << ", " << points.p5.z()  << "\n";
           file << points.p6.x() << ", " << points.p6.y() << ", " << points.p6.z()  << "\n";
        } catch (const std::ofstream::failure &err) {
            ROS_ERROR_STREAM("[ArmToLaserScannerCalibration] writeCalibrationPointsToFile Error: " << err.what());
        }
    }

    static iiwa_msgs::SplineSegment getSplineSegment (Eigen::Matrix4d h_S,std::string frameId,  int type = iiwa_msgs::SplineSegment::SPL) {
        Eigen::Quaterniond quaternion = toQuaternion(h_S);

        iiwa_msgs::SplineSegment segment;
        // Segment type
        segment.type = type;
        // Header
        segment.point.poseStamped.header.frame_id = frameId;
        // Pose
        segment.point.poseStamped.pose.position.x = h_S(0,3);
        segment.point.poseStamped.pose.position.y = h_S(1,3);
        segment.point.poseStamped.pose.position.z = h_S(2,3);
        // Orientation
        segment.point.poseStamped.pose.orientation.x = quaternion.x();
        segment.point.poseStamped.pose.orientation.y = quaternion.y();
        segment.point.poseStamped.pose.orientation.z = quaternion.z();
        segment.point.poseStamped.pose.orientation.w = quaternion.w();
        // Redundancy
        segment.point.redundancy.status = -1;
        segment.point.redundancy.turn = -1;

        return segment;
    }

    static void writeEulerZYXAndTranslationToFile(const std::string &fileName,
                                 Eigen::Vector3d eulerAnglesZYX, Eigen::Matrix4d homMatrix) {
        std::string nameAndTyp = fileName + ".txt";
        try {
            std::ofstream file(nameAndTyp);
            file << "Trans X: \t" << homMatrix(0,3) <<  "\n" << "Trans Y: \t" << homMatrix(1,3) <<  "\n" << "Trans x: \t" << homMatrix(2,3) <<  "\n" << "Roll : \t" << eulerAnglesZYX(0) <<  "\n" << "Pitch : \t" << eulerAnglesZYX(1) << "\n" << "Yaw : \t" << eulerAnglesZYX(2) << "\n";
        } catch (const std::ofstream::failure &err) {
            ROS_ERROR_STREAM("[ArmToLaserScannerCalibration] writeMatrixToFile Error: " << err.what());
        }

    }

    static void printEulerZYXAndTranslation(Eigen::Vector3d eulerAnglesZYX, Eigen::Matrix4d homMatrix) {
        ROS_INFO_STREAM("[ArmToLaserScannerCalibration] Results:" );
        ROS_INFO_STREAM("[ArmToLaserScannerCalibration] Trans X: \t" <<  homMatrix(0,3));
        ROS_INFO_STREAM("[ArmToLaserScannerCalibration] Trans Y: \t" <<  homMatrix(1,3));
        ROS_INFO_STREAM("[ArmToLaserScannerCalibration] Trans Z: \t" <<  homMatrix(2,3));
        ROS_INFO_STREAM("[ArmToLaserScannerCalibration] Roll:    \t" << eulerAnglesZYX(0));
        ROS_INFO_STREAM("[ArmToLaserScannerCalibration] Pitch:   \t" << eulerAnglesZYX(1));
        ROS_INFO_STREAM("[ArmToLaserScannerCalibration] Yaw:     \t" << eulerAnglesZYX(2));
    }

    static double median(std::vector<float> &v)
    {
        size_t n = v.size() / 2;
        nth_element(v.begin(), v.begin()+n, v.end());
        return v[n];
    }

};


#endif