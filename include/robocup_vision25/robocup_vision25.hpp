#ifndef ROBOCUP_VISION25_HPP
#define ROBOCUP_VISION25_HPP

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <image_transport/image_transport.hpp>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "../YoloV4/YoloV4.hpp"
#include "../pan_tilt/pan_tilt.hpp"
#include "dynamixel_rdk_msgs/msg/dynamixel_bulk_read_msgs.hpp"
#include "dynamixel_rdk_msgs/msg/dynamixel_control_msgs.hpp"
#include "dynamixel_rdk_msgs/msg/dynamixel_msgs.hpp"
#include "dynamixel_rdk_msgs/msg/dynamixel_status_msgs.hpp"
#include "humanoid_interfaces/msg/master2vision25.hpp"
#include "humanoid_interfaces/msg/robocupvision25.hpp"
#include "humanoid_interfaces/msg/robocupvision25feature.hpp"

#define DEG2RAD (M_PI / 180)

using namespace cv;
using namespace std;

struct HSV {
  int hmin;
  int hmax;
  int smin;
  int smax;
  int vmin;
  int vmax;
  int dilate;
  int erode;
};

typedef struct CamIntrParameter {
  double fx;
  double fy;

  double cx;
  double cy;

  CamIntrParameter() {
    fx = 0;
    fy = 0;

    cx = 0;
    cy = 0;
  }
} CamIntrParameter;

typedef struct ObjectPos {
  double dist;
  double theta;

  ObjectPos() {
    dist = 0;
    theta = 0;
  }
} ObjectPos;

class robocup_vision25 : public rclcpp::Node {
 public:
  robocup_vision25();
  ~robocup_vision25();

  PAN_TILT pan_tilt;

  cv::Mat img;
  cv::Mat info;

  Mat K_M = Mat::zeros(3, 3, CV_64FC1);

  Mat D_M = Mat::zeros(1, 5, CV_64FC1);

  Mat R_M = Mat::zeros(3, 3, CV_64FC1);

  Mat P_M = Mat::zeros(3, 4, CV_64FC1);

  Mat NEW_K_M = Mat::zeros(3, 3, CV_64FC1);

  cv::Point2d new_focalLen;
  cv::Point2d new_prncPt;

  vector<Point2f> pts, condis;
  vector<String> data;

  // For calibration
  cv::Point2d focalLen;
  cv::Point2d prncPt;
  void caminfo_open(const cv::Mat& info) {
    Mat camMat(info);
    CamIntrParameter camInfo;

    camInfo.fx = camMat.at<double>(0, 0);
    camInfo.fy = camMat.at<double>(1, 1);
    camInfo.cx = camMat.at<double>(0, 2);
    camInfo.cy = camMat.at<double>(1, 2);

    focalLen = cv::Point2d(camInfo.fx, camInfo.fy);
    prncPt = cv::Point2d(camInfo.cx, camInfo.cy);
  }

  char str_list[6];
  string str_name;
  int save_img_count = 0;

  int ball_filter_x[30] = {
      0,
  };
  int ball_filter_y[30] = {
      0,
  };
  int ball_filter_cnt = 0;
  int ball_filter_idx = 0;

  void calibration_info() {
    String temp;
    double dtemp;
    double mtemp;
    double asc46;
    double K[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    double D[5] = {0, 0, 0, 0, 0};
    double R[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    double P[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int cnt;
    ifstream Calibration_Info(
        "/home/robit/colcon_ws/src/ocam_ros2/config/camera.yaml");
    if (Calibration_Info.is_open()) {
      for (int i = 0; i < 12; i++) {
        Calibration_Info >> temp;
      }
      for (int i = 0; i < 9; i++) {
        char TEXT[256];
        dtemp = 0;
        asc46 = 1;
        mtemp = 1;
        Calibration_Info >> temp;
        strcpy(TEXT, temp.c_str());
        for (std::string::size_type j = 0; j < temp.length(); j++) {
          if (TEXT[j] == 45) {
            mtemp = -1;
          }
          if ((TEXT[j] >= 48 && TEXT[j] <= 57) || TEXT[j] == 46) {
            if (TEXT[j] == 46 || asc46 != 1) {
              if (TEXT[j] != 46) {
                dtemp += (TEXT[j] - 48) / asc46;
              }
              asc46 *= 10;
            } else {
              dtemp = (dtemp * 10) + (TEXT[j] - 48);
            }
          }
        }
        K[i] = mtemp * dtemp;
      }
      cnt = 0;
      for (int i = 0; i < K_M.rows; i++) {
        for (int j = 0; j < K_M.cols; j++) {
          K_M.at<double>(i, j) = K[cnt];
          cnt += 1;
        }
      }
      // cout<<K_M<<endl;
      // cout<<K[0]<<" "<<K[1]<<" "<<K[2]<<" "<<K[3]<<" "<<K[4]<<" "<<K[5]<<"
      // "<<K[6]<<" "<<K[7]<<" "<<K[8]<<endl;
      for (int i = 0; i < 8; i++) {
        Calibration_Info >> temp;
      }
      for (int i = 0; i < 5; i++) {
        char TEXT[256];
        dtemp = 0;
        asc46 = 1;
        mtemp = 1;
        Calibration_Info >> temp;
        strcpy(TEXT, temp.c_str());
        for (std::string::size_type j = 0; j < temp.length(); j++) {
          if (TEXT[j] == 45) {
            mtemp = -1;
          }
          if ((TEXT[j] >= 48 && TEXT[j] <= 57) || TEXT[j] == 46) {
            if (TEXT[j] == 46 || asc46 != 1) {
              if (TEXT[j] != 46) {
                dtemp += (TEXT[j] - 48) / asc46;
              }
              asc46 *= 10;
            } else {
              dtemp = (dtemp * 10) + (TEXT[j] - 48);
            }
          }
        }
        D[i] = mtemp * dtemp;
      }
      cnt = 0;
      for (int i = 0; i < D_M.rows; i++) {
        for (int j = 0; j < D_M.cols; j++) {
          D_M.at<double>(i, j) = D[cnt];
          cnt += 1;
        }
      }
      // cout<<D_M<<endl;
      // cout<<D[0]<<" "<<D[1]<<" "<<D[2]<<" "<<D[3]<<" "<<D[4]<<endl;
      for (int i = 0; i < 6; i++) {
        Calibration_Info >> temp;
      }
      for (int i = 0; i < 9; i++) {
        char TEXT[256];
        dtemp = 0;
        asc46 = 1;
        mtemp = 1;
        Calibration_Info >> temp;
        strcpy(TEXT, temp.c_str());
        for (std::string::size_type j = 0; j < temp.length(); j++) {
          if (TEXT[j] == 45) {
            mtemp = -1;
          }
          if ((TEXT[j] >= 48 && TEXT[j] <= 57) || TEXT[j] == 46) {
            if (TEXT[j] == 46 || asc46 != 1) {
              if (TEXT[j] != 46) {
                dtemp += (TEXT[j] - 48) / asc46;
              }
              asc46 *= 10;
            } else {
              dtemp = (dtemp * 10) + (TEXT[j] - 48);
            }
          }
        }
        R[i] = mtemp * dtemp;
      }
      cnt = 0;
      for (int i = 0; i < R_M.rows; i++) {
        for (int j = 0; j < R_M.cols; j++) {
          R_M.at<double>(i, j) = R[cnt];
          cnt += 1;
        }
      }
      // cout<<R_M<<endl;
      // cout<<R[0]<<" "<<R[1]<<" "<<R[2]<<" "<<R[3]<<" "<<R[4]<<" "<<R[5]<<"
      // "<<R[6]<<" "<<R[7]<<" "<<R[8]<<endl;
      for (int i = 0; i < 6; i++) {
        Calibration_Info >> temp;
      }
      for (int i = 0; i < 12; i++) {
        char TEXT[256];
        dtemp = 0;
        asc46 = 1;
        mtemp = 1;
        Calibration_Info >> temp;
        strcpy(TEXT, temp.c_str());
        for (std::string::size_type j = 0; j < temp.length(); j++) {
          if (TEXT[j] == 45) {
            mtemp = -1;
          }
          if ((TEXT[j] >= 48 && TEXT[j] <= 57) || TEXT[j] == 46) {
            if (TEXT[j] == 46 || asc46 != 1) {
              if (TEXT[j] != 46) {
                dtemp += (TEXT[j] - 48) / asc46;
              }
              asc46 *= 10;
            } else {
              dtemp = (dtemp * 10) + (TEXT[j] - 48);
            }
          }
        }
        P[i] = mtemp * dtemp;
      }
      cnt = 0;
      for (int i = 0; i < P_M.rows; i++) {
        for (int j = 0; j < P_M.cols; j++) {
          P_M.at<double>(i, j) = P[cnt];
          cnt += 1;
        }
      }
      // cout<<P_M<<endl;
      // cout<<P[0]<<" "<<P[1]<<" "<<P[2]<<" "<<P[3]<<" "<<P[4]<<" "<<P[5]<<"
      // "<<P[6]<<" "<<P[7]<<" "<<P[8]<<" "<<P[9]<<" "<<P[10]<<" "<<P[11]<<endl;
    }
    Calibration_Info.close();
    NEW_K_M = getOptimalNewCameraMatrix(K_M, D_M, Size(640, 480), 1,
                                        Size(640, 480), 0);
    cout << NEW_K_M << endl;

    new_focalLen =
        cv::Point2d(NEW_K_M.at<double>(0, 0), NEW_K_M.at<double>(1, 1));
    new_prncPt =
        cv::Point2d(NEW_K_M.at<double>(0, 2), NEW_K_M.at<double>(1, 2));
  }

  const ObjectPos calcObjectDistance(double tilt, double h,
                                     const cv::Point2d& focalLength,
                                     const cv::Point2d& principalPt,
                                     const cv::Point2d& pixelPt) {
#define DEG2RAD (M_PI / 180)
#define RAD2DEG (180 / M_PI)

    //    cout << "********* calcObjectDistance *********" << endl;
    //    cout << "tilt = " << tilt << endl;
    //    cout << "h = " << h << endl;

    ObjectPos obj;

    cv::Point2d normPt;
    //    cout << "object: " << pixelPt.x << " " << pixelPt.y << endl;
    //    cout << "principal: " << principalPt.x << " " << principalPt.y <<
    //    endl; cout << "focal length: " << focalLength.x << " " <<
    //    focalLength.y << endl;
    normPt.x = (pixelPt.x - principalPt.x) / focalLength.x;
    normPt.y = (pixelPt.y - principalPt.y) / focalLength.y;
    double u = normPt.x;
    double v = normPt.y;

    //    cout << "u = " << u << endl;
    //    cout << "v = " << v << endl;

    //    cout << "atan(v) = " << atan(v) * RAD2DEG << endl;

    double CC_ = h;
    double C_P_ = CC_ * tan((M_PI / 2) + tilt * DEG2RAD - atan(v));
    double CP_ = sqrt(CC_ * CC_ + C_P_ * C_P_);
    double Cp_ = sqrt(1 + v * v);
    double PP_ = u * CP_ / Cp_;

    //    cout << "CC_ = " << CC_ << endl;
    //    cout << "C_P_ = " << C_P_ << endl;
    //    cout << "CP_ = " << CP_ << endl;
    //    cout << "Cp_ = " << Cp_ << endl;
    //    cout << "PP_ = " << PP_ << endl;

    obj.dist = sqrt(C_P_ * C_P_ + PP_ * PP_);
    obj.theta = -atan2(PP_, C_P_) * RAD2DEG;

    //    cout << "obj.dist = " << obj.dist << endl;

    return obj;
  }

  YOLOV4::YoloV4 yolov4 = YOLOV4::YoloV4();

  int cam_nice_point = 1;

  int setting_flag = -1;
  int ball_cam_X = 0;
  int ball_cam_Y = 0;
  ObjectPos ballPos;
  ObjectPos linePos;
  ObjectPos robotPos;
  Mat Image_processing(const cv::Mat& img);

  // For timer
  int filter_cnt = 0;
  int fps_cnt = 0;
  int nice_cnt = 0;

  // For filter
  double fst_filter_x = 0, fst_filter_y = 0;
  int fst_filter_cnt = 0;
  double sec_filter_x = 0, sec_filter_y = 0;
  int sec_filter_cnt = 0;
  double final_filter_x = 0;
  double final_filter_y = 0;
  double ball_speed_vec_x = 0;
  double ball_speed_vec_y = 0;
  double ball_speed_level = 0;

  // For line
  int edgecount;
  struct VISION_POINT {
    int NUM;
    int POINT_VEC_X;
    int POINT_VEC_Y;
  };
  VISION_POINT vision_point;
  vector<VISION_POINT> vision_point_vect;
  vector<int> featkind;
  vector<Point2f> edgepoint;
  vector<Point2f> featPoint;
  vector<Point2f> nomalLinePoint;
  void edge_line_point_detect(const cv::Mat& warp_res,
                              const cv::Mat& result_mat,
                              const cv::Mat& warp_mat2);
  void FeaturePointEctraction(const cv::Mat& warp_res,
                              const cv::Mat& result_img,
                              const cv::Mat& warp_mat2, const cv::Mat& skel);
  void normalLine(const cv::Mat& line_mat, const cv::Mat& result_mat,
                  const cv::Mat& warp_mat2, const cv::Mat& warp_mat);
  void filter_100ms();

  // For Ros Topic
  void publish_vision_msg();
  void publish_localization_msg();

  humanoid_interfaces::msg::Robocupvision25 visionMsg;
  humanoid_interfaces::msg::Robocupvision25feature vision_feature_Msg;

 private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void master_callback(
      const humanoid_interfaces::msg::Master2vision25::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      info_subscription_;
  rclcpp::Subscription<humanoid_interfaces::msg::Master2vision25>::SharedPtr
      visionSub;
  std::string image_topic;

  rclcpp::TimerBase::SharedPtr timer_;
  void timerCallback();

  rclcpp::Publisher<humanoid_interfaces::msg::Robocupvision25>::SharedPtr
      visionPub;
  rclcpp::Publisher<humanoid_interfaces::msg::Robocupvision25feature>::SharedPtr
      vision_feature_Pub;
};

#endif  // ROBOCUP_VISION25_HPP
