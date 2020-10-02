//
// Created by hyj on 18-1-19.
//

#ifndef IMUSIMWITHPOINTLINE_UTILITIES_H
#define IMUSIMWITHPOINTLINE_UTILITIES_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <vector>
#include "imu.h"

// save 3d points to file
void save_points(
    std::string filename,
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >
        points);

// save 3d points and it's obs in image
void save_features(
    std::string filename,
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >
        points,
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >
        features,
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >
        features_pixel,
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >
        features_velocity);

// save line obs
void save_lines(
    std::string filename,
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >
        features);

void LoadPose(std::string filename, std::vector<MotionData>& pose);

// save imu body data
void save_Pose(std::string filename, std::vector<MotionData> pose);

// save pose as TUM style
void save_Pose_asTUM(std::string filename, std::vector<MotionData> pose);

#endif  // IMUSIMWITHPOINTLINE_UTILITIES_H
