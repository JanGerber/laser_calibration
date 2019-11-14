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

    static inline geometry_msgs::PoseStamped
    toPoseStampedMsg(const Eigen::Matrix4d &h_S, const std_msgs::Header &header) {
        geometry_msgs::PoseStamped msg;
        msg.header = header;
        msg.pose = toPoseMsg(h_S);
        return msg;
    }

    static Eigen::Matrix3d toEigenMatrix(geometry_msgs::Quaternion quaternion) {
        Eigen::Quaterniond eigenQuaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        return eigenQuaternion.toRotationMatrix();
    }

};


#endif