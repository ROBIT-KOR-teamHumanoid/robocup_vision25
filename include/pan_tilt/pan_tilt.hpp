#ifndef PAN_TILT_H
#define PAN_TILT_H

#include <iostream>
#include <memory>
#include <random>

#include "dynamixel_rdk_msgs/msg/dynamixel_bulk_read_msgs.hpp"
#include "dynamixel_rdk_msgs/msg/dynamixel_control_msgs.hpp"
#include "dynamixel_rdk_msgs/msg/dynamixel_msgs.hpp"
#include "dynamixel_rdk_msgs/msg/dynamixel_status_msgs.hpp"
#include "rclcpp/rclcpp.hpp"

#define ROBOT_HEIGHT 600  // 490//475// 505//545// 755.0//544
#define TILT_L 0          // 65.0//75.0
#define TILT_D -49
#define DEG2RAD (M_PI / 180)
#define RAD2DEG (180 / M_PI)
#define PAN_MAX 150
#define PAN_MIN -150
#define TILT_MAX 0
#define TILT_MIN -90

using namespace std;

class PAN_TILT : public rclcpp::Node {
 public:
  PAN_TILT() : Node("pan_tilt_node") {
    id = 32;
    ptpos.PAN_POSITION = 0;
    ptpos.TILT_POSITION = -45;
    Motor_Pub = this->create_publisher<dynamixel_rdk_msgs::msg::DynamixelMsgs>(
        "pan_dxl", 10);
  }

  PAN_TILT(int id_, double pan_init_) : Node("pan_tilt_node") {
    id = id_;
    ptpos.PAN_POSITION = pan_init_;
    ptpos.TILT_POSITION = 2048;
    Motor_Pub = this->create_publisher<dynamixel_rdk_msgs::msg::DynamixelMsgs>(
        "pan_dxl", 10);
  }

  // For Scan
  int Scan_index = 1;  // 0 ~ (size of Scan_level) - 1
  int temp_index = 1;
  double Scan_level[4] = {-0.872665, 0, 0.872665, 0};  // 카메라 각도
  int Scan_timer = 0;
  int Scan_stop_time = 50;  // 스캔속도 설정
  int Scan_nice_time = 5;   // 카메라가 움직이기 직전 몇장을 이미지처리 할건지
  int Scan_nice_weight = 0;

  struct POSITION {
    double PAN_POSITION = 0;
    double TILT_POSITION = 0;
  };
  POSITION ptpos;

  // For Tracking
  double target_pan = 0, target_tilt = 0;
  double target_x = 0, target_y = 0;
  double target_absx = 0, target_absy = 0;

  int tracking_cnt = 0;
  int no_ball_cnt = 0;

  int error = -1;

  int mode(int value) {
    // PRE CONDITION : value
    // POST CONDITION : cam_nice_point
    // PURPOSE : 로봇의 상태에 따라 팬틸트 제어

    if (value != 2 && value != 3) {
      no_ball_cnt = 100;
      tracking_cnt = 0;
    }
    switch (value) {
      case 1: {
        // Oper_pan_tilt
        send_ptmsg();
        return 1;
        break;
      }
      case 2: {
        // //case2!!!!!!!!!!!!!!!!!
        // //Ball 스캔
        if (tracking_cnt > 0)  // 공이 잡혀서 트래킹
        {
          if (target_x != 0 || target_y != 0) {
            Tracking_what();  // 공 위치로 트래킹
            no_ball_cnt = 0;
          } else {
            tracking_cnt = 0;
          }
          if (Scan_index == 1) {
            Scan_nice_weight += 1;
          }  // 로봇이 정면을 보고
          else {
            Scan_nice_weight = 0;
          }
          if (Scan_nice_weight >= 5) {
            return 1;
          }  // 로봇이 정면을 보고 있는
          return 2;  // 로봇이 정면을 보고 있는 경우 2 반환

        } else if (no_ball_cnt > 30)  // 공이 특정 시간동안 안잡힘
        {
          ptpos.PAN_POSITION = Scan_level[Scan_index];
          send_ptmsg();
          Scan_timer += 1;
          if (Scan_timer >= Scan_stop_time)  // 스캔한 시간이 임계값을 넘겼을
          {
            Scan_timer = 0;
            Scan_index += 1;  // 스캔 각도 이동
            if (Scan_index >= 4) {
              Scan_index = 0;
            }
          }
          if (Scan_timer >= Scan_stop_time - Scan_nice_time) {
            return 1;
          }
          // return 1;
        } else  // 공이 잡히다가 안잡힘
        {
          ptpos.PAN_POSITION = Scan_level[Scan_index];
          send_ptmsg();
          return 1;
        }

        // ptpos.PAN_POSITION = 0;
        // send_ptmsg();
        // return 1;
        // break;
      }
      case 3: {
        ptpos.PAN_POSITION = 0;
        send_ptmsg();
        return 1;
        break;
      }
      case 4: {
        // Master_Callback
        send_ptmsg();
        Scan_index = 1;
        return 1;
        break;
      }
      case 99: {
        // //ERROR
        // error *= -1;
        // ptpos.PAN_POSITION = error * 100;
        // ptpos.TILT_POSITION = -45;
        // send_ptmsg();
        // break;
      }
      default: {
        init();
        break;
      }
    }
    return 0;
  }

 protected:
 private:
  rclcpp::Publisher<dynamixel_rdk_msgs::msg::DynamixelMsgs>::SharedPtr
      Motor_Pub;
  dynamixel_rdk_msgs::msg::DynamixelMsgs pan_msg;

  int id = 32;
  float velocity = 1;
  float acceleration = -1;

  void send_ptmsg() {
    if (Scan_index != temp_index) {
      ptpos.PAN_POSITION = Scan_level[Scan_index];
      temp_index = Scan_index;
    } else {
      return;
    }
    publish_motor_msg();
  }

  void init() {
    ptpos.PAN_POSITION = 0;
    ptpos.TILT_POSITION = -45;
    for (int i = 0; i < 5; i++) {
      send_ptmsg();
    }
  }

  void Tracking_what() {
    // PRE CONDITION : absx, absy
    // POST CONDITION : Tracking
    // PURPOSE : 스캔할 물체의 위치에 따라 어떤 각도로 팬을 조절할지 정함

    int dis =
        sqrt(pow(target_absx, 2) + pow(target_absy, 2));  // 물체와 거리 계산
    // cout<<dis<<endl;
    if (dis > 500)  // 거리가 500이 넘지않으면 실행
    {
      int angle = atan2(target_absy, target_absx) * RAD2DEG -
                  90;  // 타겟 과의 각도 계산

      // 각도 예외처리
      if (angle > 180) {
        angle -= 360;
      } else if (angle < -180) {
        angle += 360;
      }
      // cout<<angle<<endl;

      //+-50의 임계값에 따라 각도 설정
      if (angle < -50) {
        Scan_index = 2;
      } else if (angle > 50) {
        Scan_index = 0;
      } else {
        Scan_index = 1;
      }
    } else {
      Scan_index = 1;
    }  // 거리가 500이 넘으면 팬 각도 정면으로 설정
    ptpos.PAN_POSITION = Scan_level[Scan_index];  // 모터 제어를 위해 값 전송
    send_ptmsg();                                 // 팬틸트 데이터 msg 전송
  }

  void publish_motor_msg() {
    // PRE CONDITION : pan_tilt
    // POST CONDITION : pan_tilt
    // PURPOSE : 로봇의 팬틸트 모터를 제어하기 위한 데이터 PUBLISH

    pan_msg.goal_position = ptpos.PAN_POSITION;
    pan_msg.profile_velocity = 0;
    pan_msg.profile_acceleration = 0;

    Motor_Pub->publish(pan_msg);
  }
};

#endif  // PAN_TILT_H
