//
// Created by hyj on 17-6-22.
//

#include <sys/stat.h>
#include <fstream>
#include <random>
#include "../src/imu.h"
#include "../src/param.h"
#include "../src/utilities.h"
#include "time.h"

using Point = Eigen::Vector4d;
using Points = std::vector<Point, Eigen::aligned_allocator<Point> >;
using Line = std::pair<Eigen::Vector4d, Eigen::Vector4d>;
using Lines = std::vector<Line, Eigen::aligned_allocator<Line> >;

void CreatePointsLines(Points& points, Lines& lines) {
  std::ifstream f;
  f.open("../bin/house_model/house.txt");

  while (!f.eof()) {
    std::string s;
    std::getline(f, s);
    if (!s.empty()) {
      std::stringstream ss;
      ss << s;
      double x, y, z;
      ss >> x;
      ss >> y;
      ss >> z;
      Eigen::Vector4d pt0(x, y, z, 1);
      ss >> x;
      ss >> y;
      ss >> z;
      Eigen::Vector4d pt1(x, y, z, 1);

      bool isHistoryPoint = false;
      for (int i = 0; i < points.size(); ++i) {
        Eigen::Vector4d pt = points[i];
        if (pt == pt0) {
          isHistoryPoint = true;
        }
      }
      if (!isHistoryPoint) points.push_back(pt0);

      isHistoryPoint = false;
      for (int i = 0; i < points.size(); ++i) {
        Eigen::Vector4d pt = points[i];
        if (pt == pt1) {
          isHistoryPoint = true;
        }
      }
      if (!isHistoryPoint) points.push_back(pt1);

      // pt0 = Twl * pt0;
      // pt1 = Twl * pt1;
      lines.emplace_back(pt0, pt1);  // lines
    }
  }
  std::cout << "read house model success!" << std::endl;
  // std::cout << "all points:" << std::endl;
  // for (int i = 0; i < points.size(); ++i) {
  //   std::cout << "point:" << i << ":" << points[i].transpose() << std::endl;
  // }
  // std::cout << "all lines:" << std::endl;
  // for (int i = 0; i < lines.size(); ++i) {
  //   std::cout << "line " << i << ":" << lines[i].first.transpose() << "--"
  //             << lines[i].second.transpose() << std::endl;
  // }
  // create more 3d points, you can comment this code
  int n = points.size();
  for (int j = 0; j < n; ++j) {
    Eigen::Vector4d p = points[j] + Eigen::Vector4d(0.5, 0.5, -0.5, 0);
    points.push_back(p);
  }
  // save points
  save_points("../bin/all_points.txt", points);
}

int main() {
  // Eigen::Quaterniond Qwb;
  // Qwb.setIdentity();
  // Eigen::Vector3d omega (0,0,M_PI/10);
  // double dt_tmp = 0.005;
  // for (double i = 0; i < 20.; i += dt_tmp) {
  //     Eigen::Quaterniond dq;
  //     Eigen::Vector3d dtheta_half =  omega * dt_tmp /2.0;
  //     dq.w() = 1;
  //     dq.x() = dtheta_half.x();
  //     dq.y() = dtheta_half.y();
  //     dq.z() = dtheta_half.z();
  //     Qwb = Qwb * dq;
  // }
  // std::cout << Qwb.coeffs().transpose() <<"\n"<<Qwb.toRotationMatrix() <<
  // std::endl;

  // 建立keyframe文件夹
  mkdir("../bin/keyframe", 0777);

  // 生成3d points
  Points points;
  Lines lines;
  CreatePointsLines(points, lines);

  // IMU model
  Param params;
  IMU imuGen(params);

  // create imu data
  // imu pose gyro acc
  std::vector<MotionData> imudata;
  std::vector<MotionData> imudata_noise;
  for (float t = params.t_start; t < params.t_end;) {
    MotionData data = imuGen.MotionModel(t);
    imudata.push_back(data);

    // add imu noise
    MotionData data_noise = data;
    imuGen.addIMUnoise(data_noise);
    imudata_noise.push_back(data_noise);

    t += 1.0 / params.imu_frequency;
  }
  imuGen.init_velocity_ = imudata[0].imu_velocity;
  imuGen.init_twb_ = imudata.at(0).twb;
  imuGen.init_Rwb_ = imudata.at(0).Rwb;
  save_Pose("../bin/imu_pose.txt", imudata);
  save_Pose("../bin/imu_pose_noise.txt", imudata_noise);

  imuGen.testImu(
      "../bin/imu_pose.txt",
      "../bin/imu_int_pose.txt");  // test the imu data, integrate the imu
                                   // data to generate the imu trajecotry
  imuGen.testImu("../bin/imu_pose_noise.txt", "../bin/imu_int_pose_noise.txt");

  // cam pose
  std::vector<MotionData> camdata;
  for (float t = params.t_start; t < params.t_end;) {
    MotionData imu =
        imuGen.MotionModel(t);  // imu body frame to world frame motion
    //imuGen.addIMUnoise(imu);    // add niose
    MotionData cam;

    cam.timestamp = imu.timestamp;
    cam.Rwb = imu.Rwb * params.R_bc;  // cam frame in world frame
    cam.twb = imu.twb +
              imu.Rwb * params.t_bc;  //  Tcw = Twb * Tbc ,  t = Rwb * tbc + twb

    camdata.push_back(cam);
    t += 1.0 / params.cam_frequency;
  }
  save_Pose("../bin/cam_pose.txt", camdata);
  save_Pose_asTUM("../bin/cam_pose_tum.txt", camdata);

  std::default_random_engine random(time(NULL));
  std::uniform_real_distribution<double> pixel_rondom(-params.pixel_noise,
                                                      params.pixel_noise);
  // 保存上一帧的特征点值
  std::vector<Eigen::Vector2d> pre_norm_pts;
  // points obs in image
  for (int n = 0; n < camdata.size(); ++n) {
    MotionData data = camdata[n];
    Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
    Twc.block(0, 0, 3, 3) = data.Rwb;
    Twc.block(0, 3, 3, 1) = data.twb;
    if (n == 0) {
      // std::cout << "Twc:" << Twc << std::endl;
    }

    // 遍历所有的特征点，看哪些特征点在视野里
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >
        points_cam;  // ３维点在当前cam视野里
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >
        features_cam;  // 对应的２维图像坐标
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >
        features_pixel;  // 对应的２维图像坐标
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >
        features_velocity;  // 对应的２维坐标点速度
    pre_norm_pts.clear();
    for (int i = 0; i < points.size(); ++i) {
      Eigen::Vector4d pw = points[i];  //
      pw[3] = 1;                       //改成齐次坐标最后一位
      Eigen::Vector4d pc1 =
          Twc.inverse() * pw;  // T_wc.inverse() * Pw  -- > point in cam frame

      if (pc1(2) < 0) {
        std::cout << "remove points" << std::endl;
        continue;  // z必须大于０,在摄像机坐标系前方
      }

      Eigen::Vector2d obs(pc1(0) / pc1(2), pc1(1) / pc1(2));
      Eigen::Vector2d pixel;
      double u = params.fx * obs[0] + params.cx;
      double v = params.fy * obs[1] + params.cy;
      pixel << u, v;

      // add pixle noise
      // std::cout << "pixel noise:" << pixel_rondom(random) << std::endl;
      u = u + pixel_rondom(random);
      v = v + pixel_rondom(random);
      pixel << u, v;

      double x = (u - params.cx) / params.fx;
      double y = (v - params.cy) / params.fy;
      obs << x, y;
      // 计算像素速度
      Eigen::Vector2d velocity;
      if (n == 0) {
        velocity << 0.0, 0.0;
      } else {
        double dt = camdata[n].timestamp - camdata[n - 1].timestamp;
        double x_v = (x - pre_norm_pts[i][0]) / dt;
        double y_v = (y - pre_norm_pts[i][1]) / dt;
        velocity << x_v, y_v;
      }
      if ((obs(0) * 460 + 255) < params.image_h && (obs(0) * 460 + 255) > 0 &&
          (obs(1) * 460 + 255) > 0 && (obs(1) * 460 + 255) < params.image_w) {
        points_cam.push_back(points[i]);
        // std::cout << "points[i]:" << points[i].transpose() << std::endl;
        features_cam.push_back(obs);
        features_pixel.push_back(pixel);
        features_velocity.emplace_back(velocity);
      }
      pre_norm_pts.emplace_back(obs);
    }
    // save points
    std::stringstream filename1;
    filename1 << "../bin/keyframe/all_points_" << n << ".txt";
    save_features(filename1.str(), points_cam, features_cam, features_pixel,
                  features_velocity);
  }

  // // lines obs in image
  // for (int n = 0; n < camdata.size(); ++n) {
  //   MotionData data = camdata[n];
  //   Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
  //   Twc.block(0, 0, 3, 3) = data.Rwb;
  //   Twc.block(0, 3, 3, 1) = data.twb;

  //   // 遍历所有的特征点，看哪些特征点在视野里
  //   // std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>
  //   >
  //   // points_cam;    // ３维点在当前cam视野里
  //   std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >
  //       features_cam;  // 对应的２维图像坐标
  //   for (int i = 0; i < lines.size(); ++i) {
  //     Line linept = lines[i];

  //     Eigen::Vector4d pc1 =
  //         Twc.inverse() *
  //         linept.first;  // T_wc.inverse() * Pw  -- > point in cam frame
  //     Eigen::Vector4d pc2 =
  //         Twc.inverse() *
  //         linept.second;  // T_wc.inverse() * Pw  -- > point in cam frame

  //     if (pc1(2) < 0 || pc2(2) < 0) continue;  //
  //     z必须大于０,在摄像机坐标系前方

  //     Eigen::Vector4d obs(pc1(0) / pc1(2), pc1(1) / pc1(2), pc2(0) / pc2(2),
  //                         pc2(1) / pc2(2));
  //     // if(obs(0) < params.image_h && obs(0) > 0 && obs(1)> 0 && obs(1) <
  //     // params.image_w)
  //     { features_cam.push_back(obs); }
  //   }

  //   // save points
  //   std::stringstream filename1;
  //   filename1 << "../bin/keyframe/all_lines_" << n << ".txt";
  //   save_lines(filename1.str(), features_cam);
  // }

  return 0;
}
