#include "../include/robocup_vision25/robocup_vision25.hpp"

using namespace cv;
using namespace std;

HSV fieldhsv, linehsv;

int scan_value = 2;

// robot
int robot_absx = 0;
int robot_absy = 0;

// save
int save_img_flag = 0;
int save_img_start = 0;
int save_img_timer = 0;

// error
int ocam_error = 0;

robocup_vision25::robocup_vision25() : Node("robocup_vision25") {
  image_topic =
      this->declare_parameter<std::string>("image_topic", "/camera/image_raw");

  image_topic = this->get_parameter("image_topic").as_string();

  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic, 10,
      std::bind(&robocup_vision25::image_callback, this,
                std::placeholders::_1));

  info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", 10,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        //   K_M = cv::Mat(3, 3, CV_64F, (void *)msg->k.data());
        //   D_M = cv::Mat(1, 5, CV_64F, (void *)msg->d.data());
        //   R_M = cv::Mat(3, 3, CV_64F, (void *)msg->r.data());
        //   P_M = cv::Mat(3, 4, CV_64F, (void *)msg->p.data());
      });

  visionSub =
      this->create_subscription<humanoid_interfaces::msg::Master2vision25>(
          "master2vision", 10,
          std::bind(&robocup_vision25::master_callback, this,
                    std::placeholders::_1));

  visionPub = this->create_publisher<humanoid_interfaces::msg::Robocupvision25>(
      "vision", 10);
  vision_feature_Pub =
      this->create_publisher<humanoid_interfaces::msg::Robocupvision25feature>(
          "vision_feature", 10);
  Motor_Pub = this->create_publisher<dynamixel_rdk_msgs::msg::DynamixelMsgs>(
      "pan_dxl", 10);

  RCLCPP_INFO(this->get_logger(), "robocup_vision25 node has been started.");
  RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s",
              image_topic.c_str());

  // calibration_info();

  // focalLen(257.919929, 261.486213);

  focalLen.x = 257.919929;
  focalLen.y = 261.486213;
  prncPt.x = 314.471665;
  prncPt.y = 255.536946;

  K_M.at<double>(0, 0) = 257.919929;
  K_M.at<double>(1, 1) = 261.486213;
  K_M.at<double>(0, 2) = 314.471665;
  K_M.at<double>(1, 2) = 255.536946;
  K_M.at<double>(2, 2) = 1;
  D_M.at<double>(0, 0) = -0.171346;
  D_M.at<double>(0, 1) = 0.018811;
  D_M.at<double>(0, 2) = 0.000756;
  D_M.at<double>(0, 3) = 0.000561;
  D_M.at<double>(0, 4) = 0.000000;
  R_M.at<double>(0, 0) = 1.0;
  R_M.at<double>(1, 1) = 1.0;
  R_M.at<double>(2, 2) = 1.0;
  P_M.at<double>(0, 0) = 174.74255;
  P_M.at<double>(0, 2) = 317.73894;
  P_M.at<double>(1, 1) = 210.82758;
  P_M.at<double>(1, 2) = 266.48742;
  P_M.at<double>(2, 2) = 1.0;
  NEW_K_M.at<double>(0, 0) = 257.919929;
  NEW_K_M.at<double>(1, 1) = 261.486213;
  NEW_K_M.at<double>(0, 2) = 314.471665;
  NEW_K_M.at<double>(1, 2) = 255.536946;
  NEW_K_M.at<double>(2, 2) = 1;

  // cv::Point2d prncPt(314.471665, 255.536946);

  srand((unsigned int)time(NULL));
  for (int i = 0; i < 6; i++) {
    str_list[i] = 'a' + (rand() % 26);
  }
  string str_name(str_list);
  cout << str_list << endl;

  timer_ = this->create_wall_timer(
      500ms, std::bind(&robocup_vision25::timerCallback, this));
}

robocup_vision25::~robocup_vision25() { cv::destroyAllWindows(); }

void robocup_vision25::image_callback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  static cv::Mat frame;  // memory leak
  try {
    frame = cv_bridge::toCvShare(msg, "bgr8")->image;

    if (frame.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Received empty frame.");
      return;
    }

    if (cv::waitKey(10) == 27) {
      rclcpp::shutdown();
    }
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.",
                 msg->encoding.c_str());
  }

  // focalLen = cv::Point2d(K_M.at<double>(0, 0), K_M.at<double>(1, 1));
  // prncPt = cv::Point2d(K_M.at<double>(0, 2), K_M.at<double>(1, 2));

  cam_nice_point = pan_tilt.mode(scan_value);  // 팬틸트 모드 변경
  std::cout << "scan_value : " << scan_value << std::endl;

  Mat result_image = Image_processing(frame);

  publish_vision_msg();
  publish_motor_msg();

  cv::imshow(image_topic.c_str(), result_image);

  fps_cnt += 1;
}

void robocup_vision25::timerCallback() {
  // PRE CONDITION : filter_cnt, fst_filter_cnt, fst_filter_N, ballPos.dist
  // POST CONDITION : ball_speed_vec_N, ball_speed_level
  // PURPOSE : 이동평균 필터를 적용하여 캠의 오차 값을 보정하고 이동중인 물체를
  // 포착

  if (ocam_error) {
    scan_value = 99;
    cam_nice_point = pan_tilt.mode(scan_value);
  } else if (scan_value == 99) {
    scan_value = 0;
  }
  if (scan_value != 2 && scan_value != 3) {
    ball_speed_vec_x = 0;
    ball_speed_vec_y = 0;
    ball_speed_level = 0;
  }
  if (filter_cnt >= 9) {
    // 해당 함수는 100ms마다 실행되고 총 10번 실행시 fps계산

    // fps계산
    char TEXT[256];
    sprintf(TEXT, "FPS : %d", fps_cnt); /*ui.FPS_value->setText(TEXT);*/
    cout << "fps_cnt : " << fps_cnt << "    nice_cnt : " << nice_cnt << endl;
    if (fps_cnt == 0) {
      ocam_error = 1;
    } else {
      ocam_error = 0;
    }
    fps_cnt = 0;
    filter_cnt = 0;
    nice_cnt = 0;
  }
  if (fst_filter_cnt != 0) {
    sec_filter_x += fst_filter_x / fst_filter_cnt;
    sec_filter_y += fst_filter_y / fst_filter_cnt;
    sec_filter_cnt += 1;
    fst_filter_x = 0;
    fst_filter_y = 0;
    fst_filter_cnt = 0;
  }

  if (filter_cnt == 0 || filter_cnt == 3 || filter_cnt == 6) {
    if (sec_filter_cnt != 0) {
      ball_speed_vec_x = final_filter_x;
      ball_speed_vec_y = final_filter_y;

      final_filter_x = sec_filter_x / sec_filter_cnt;
      final_filter_y = sec_filter_y / sec_filter_cnt;
      sec_filter_x = 0;
      sec_filter_y = 0;
      sec_filter_cnt = 0;

      ball_speed_vec_x = ball_speed_vec_x - final_filter_x;
      ball_speed_vec_y = ball_speed_vec_y - final_filter_y;
      double dis = sqrt(pow(ball_speed_vec_x, 2) + pow(ball_speed_vec_y, 2));
      if (dis != 0) {
        ball_speed_vec_x /= dis;
        ball_speed_vec_y /= dis;
      } else {
        ball_speed_vec_x = 0;
        ball_speed_vec_y = 0;
      }
      if (ballPos.dist == 0) {
        ball_speed_level = 0;
      } else {
        double tempdist = ballPos.dist;
        if (tempdist > 1500) {
          tempdist = 1500;
        }
        ball_speed_level = (int)(dis / 5) * abs(1500 - tempdist) / 500;
      }
    }
  }

  filter_cnt += 1;
}

Mat robocup_vision25::Image_processing(const cv::Mat &img) {
  // !!!!!!!제일 중요한 부분!!!!!!!
  // PRE CONDITION : img, cam_nice_point,
  // POST CONDITION : ball_filter_N, ball_cam_N, fst_filter_N,
  // vision_feature_Msg, visionMsg PURPOSE : 캠이 현재 볼 트래킹 상태 일 때,
  // 욜로 데이터를 통해 객체를 검출하고 해당 객체가 공, 라인, 로봇인지 판별 한
  // 후 객체 데이터를 저장 및 다른 노드로 전송

  data.clear();  // 데이터 초기화

  // HSV는 현재 사용하지 않음. 유산 기능

  // HSV
  Mat result_mat = img.clone();
  Mat HSV_mat = img.clone();
  cv::cvtColor(HSV_mat, HSV_mat, CV_BGR2HSV);
  GaussianBlur(HSV_mat, HSV_mat, Size(15, 15), 0);

  if (cam_nice_point == 0)  // 캠이 볼트래킹 모드가 아닐 경우 INIT 상태로 설정
  {
    // 변수 초기화
    ballPos.dist = 0;
    pan_tilt.target_x = 0;
    pan_tilt.target_y = 0;
    pan_tilt.target_absx = 0;
    pan_tilt.target_absy = 0;
    ball_cam_X = 0;
    ball_cam_Y = 0;
  }

  if (cam_nice_point != 0)  // 캠이 볼트래킹 모드일 경우 욜로 상태로 설정
  {
    Rect remove_rect = Rect(0, 0, 0, 0);

    // YoloRun 함수를 통해 이미지에 있는 객체를 검출 및 판별하여 객체의 데이터를
    // vector_yolo 벡터 컨테이너에 저장
    yolov4.vector_yolo = yolov4.YoloRun(img);

    int cnt_ball = 0, cnt_line = 0, cnt_robot = 0;
    for (std::vector<YOLOV4::YOLO_RESULT>::size_type i = 0;
         i < yolov4.vector_yolo.size(); i++) {
      // 벡터 컨테이너의 크기(물체를 검출한 횟수)만큼 반복 실행하여 검출된
      // 물체가 공, 라인, 로봇 중 어떤 데이터인지 판별하고 해당 물체의 카운트에
      // +1을 더함
      if (yolov4.vector_yolo[i].NUM == 0) {
        cnt_ball += 1;
      } else if (yolov4.vector_yolo[i].NUM == 1) {
        cnt_line += 1;
      } else if (yolov4.vector_yolo[i].NUM == 2) {
        cnt_robot += 1;
      }
    }
    if (save_img_start && save_img_timer >= 25) {
      // 이미지 세이브 카운트 및 플래그, 사용 안됨
      save_img_timer = 0;
      if (cnt_ball != 0) {
        save_img_flag = 1;
      } else if (cnt_line != 0) {
        save_img_flag = 1;
      } else if (cnt_robot != 0) {
        save_img_flag = 1;
      }
    }
    save_img_timer += 1;

    // 이후 코드부터 카운트 된 물체의 수 만큼 해당 물체의 위치를 판별함
    // cal distance
    if (cnt_ball != 0)  // 공이 카운트 됐을 경우 실행
    {
      Rect result(0, 0, 0, 0);
      double score = 0;
      for (std::vector<YOLOV4::YOLO_RESULT>::size_type i = 0;
           i < yolov4.vector_yolo.size(); i++)  // 검출된 객체 수 만큼 반복 실행
      {
        if ((yolov4.vector_yolo[i].NUM == 0) &&
            (yolov4.vector_yolo[i].SCORE > score))
        // 검출된 객체가 공이고 객체의 신뢰도가 특정 값 이상일 경우 실행
        {
          // 결과 값에 해당 욜로 데이터를 저장
          result = Rect(yolov4.vector_yolo[i].X, yolov4.vector_yolo[i].Y,
                        yolov4.vector_yolo[i].W, yolov4.vector_yolo[i].H);
          remove_rect = result;  // 이후 라인 판독 시 사용될 변수

          // 신뢰도 특정 값 업데이트, 이후 해당 값보다 높은 신뢰도를 가진 객체가
          // 검출되면 해당 객체 데이터로 업데이트됨
          score = yolov4.vector_yolo[i].SCORE;
        }
      }

      // 버퍼에 공의 좌표와 넓이 및 높이 데이터 입력
      char buf[100];
      sprintf(buf, "%d %lf %lf %lf %lf", 0,
              ((double)result.x + (double)result.width / 2) / 640,
              ((double)result.y + (double)result.height / 2) / 480,
              (double)result.width / 640, (double)result.height / 480);
      string value_total(buf);
      data.push_back(value_total);

      nice_cnt += 1;  // 공이 검출될 경우 nice_cnt 에 +1

      // 공의 중앙 계산
      double tilt_param = result.height / 2 * (1 - (double(result.y) / 480));
      double ball_center_X = result.x + result.width / 2;
      double ball_center_Y = result.y + result.height / 2 + tilt_param;

      // UI 에 공 위치 그리기
      int color = score * 255;

      line(result_mat, Point(ball_center_X, ball_center_Y),
           Point(ball_center_X, ball_center_Y), Scalar(0, 0, 0), 3, 8, 0);
      circle(result_mat,
             Point(result.x + result.width / 2, result.y + result.height / 2),
             (result.width + result.height) / 4, Scalar(color, color, color), 3,
             8, 0);

      // 공의 좌표 벡터 컨테이너로 저장
      vector<Point2f> ball_pts;
      ball_pts.push_back(Point2f(ball_center_X, ball_center_Y));

      // 물체와 로봇의 거리 계산

      undistortPoints(ball_pts, ball_pts, K_M, D_M, Mat(), NEW_K_M);
      for (std::vector<cv::Point_<float>>::size_type i = 0; i < ball_pts.size();
           i++) {
        ballPos = calcObjectDistance(
            pan_tilt.ptpos.TILT_POSITION,
            ROBOT_HEIGHT +
                TILT_L * (cos(pan_tilt.ptpos.TILT_POSITION * DEG2RAD) - 1),
            focalLen, prncPt, Point(ball_pts[i].x, ball_pts[i].y));
      }
      ball_pts.clear();

      // 공에 이동평균 필터 적용
      pan_tilt.target_x = ballPos.dist * sin(ballPos.theta * M_PI / 180);
      pan_tilt.target_y = ballPos.dist * cos(ballPos.theta * M_PI / 180);
      pan_tilt.target_absx =
          pan_tilt.target_x * cos((-1) * pan_tilt.ptpos.PAN_POSITION) -
          pan_tilt.target_y * sin((-1) * pan_tilt.ptpos.PAN_POSITION);
      pan_tilt.target_absy =
          pan_tilt.target_x * sin((-1) * pan_tilt.ptpos.PAN_POSITION) +
          pan_tilt.target_y * cos((-1) * pan_tilt.ptpos.PAN_POSITION);

      if (ball_filter_cnt < 30) {
        ball_filter_x[ball_filter_cnt] = ball_center_X;
        ball_filter_y[ball_filter_cnt] = ball_center_Y;
        ball_cam_X = ball_center_X;
        ball_cam_Y = ball_center_Y;
        ball_filter_cnt++;
      } else {
        if (ball_center_X == 0 && ball_center_Y == 0) {
          ball_cam_X = ball_center_X;
          ball_cam_Y = ball_center_Y;
        } else {
          ball_filter_x[ball_filter_idx] = ball_center_X;
          ball_filter_y[ball_filter_idx] = ball_center_Y;
          for (int i = 0; i < 30; i++) {
            ball_cam_X += ball_filter_x[i];
            ball_cam_Y += ball_filter_y[i];
          }
          ball_cam_X /= 30;
          ball_cam_Y /= 30;
        }

        if (ball_filter_idx > 29) {
          ball_filter_idx = 0;
        } else {
          ball_filter_idx++;
        }
      }

      // 필터 적용 후 최종적으로 나온 데이터 저장
      ball_cam_X = ball_center_X;
      ball_cam_Y = ball_center_Y;

      // 캠의 이동평균 필터를 위한 변수 저장
      pan_tilt.tracking_cnt += 1;

      fst_filter_x += pan_tilt.target_absx;
      fst_filter_y += pan_tilt.target_absy;
      fst_filter_cnt += 1;
    } else  // 공이 카운트 되지 않았을 경우 실행
    {
      ballPos.dist = 0;
      pan_tilt.target_x = 0;
      pan_tilt.target_y = 0;
      pan_tilt.target_absx = 0;
      pan_tilt.target_absy = 0;
      ball_cam_X = 0;
      ball_cam_Y = 0;

      pan_tilt.no_ball_cnt += 1;
    }

    if (cnt_line != 0)  // 라인(특징점)이 카운트 됐을 경우 실행
    {
      for (std::vector<YOLOV4::YOLO_RESULT>::size_type i = 0;
           i < yolov4.vector_yolo.size(); i++)  // 검출된 객체 수 만큼 반복 실행
      {
        if ((yolov4.vector_yolo[i].NUM ==
             1))  // 검출된 객체가 특징점일 경우 실행
        {
          // 결과 값에 해당 욜로 데이터를 저장
          Rect result(yolov4.vector_yolo[i].X, yolov4.vector_yolo[i].Y,
                      yolov4.vector_yolo[i].W, yolov4.vector_yolo[i].H);
          double line_center_X = result.x + result.width / 2;
          double line_center_Y = result.y + result.height / 2;

          if ((line_center_X > remove_rect.x + remove_rect.width ||
               line_center_X < remove_rect.x) ||
              (line_center_Y > remove_rect.y + remove_rect.height ||
               line_center_Y < remove_rect.y))
          // 라인의 예외처리?
          {
            // 버퍼에 라인의 좌표 데이터 입력
            char buf[100];
            sprintf(buf, "%d %lf %lf %lf %lf", 1,
                    ((double)result.x + (double)result.width / 2) / 640,
                    ((double)result.y + (double)result.height / 2) / 480,
                    (double)result.width / 640, (double)result.height / 480);
            string value_total(buf);
            data.push_back(value_total);

            // UI 에 특징점 위치 그리기
            int color = yolov4.vector_yolo[i].SCORE * 255;
            line(result_mat, Point(line_center_X, line_center_Y),
                 Point(line_center_X, line_center_Y), Scalar(color, 0, 0), 10,
                 8, 0);

            // 데이터 벡터 컨테이너에 저장
            pts.push_back(Point2f(line_center_X, line_center_Y));
            condis.push_back(Point2f(yolov4.vector_yolo[i].SCORE, 0));
          }
        }
      }

      // 저장된 데이터가 존재 할 경우 undistortPoints 실행?

      if (pts.size() != 0) {
        undistortPoints(pts, pts, K_M, D_M, Mat(), NEW_K_M);
      }

      for (std::vector<cv::Point_<float>>::size_type i = 0; i < pts.size();
           i++)  // 저장된 데이터 수 만큼 반복
      {
        // 특징점의 거리 계산
        linePos = calcObjectDistance(
            pan_tilt.ptpos.TILT_POSITION,
            ROBOT_HEIGHT +
                TILT_L * (cos(pan_tilt.ptpos.TILT_POSITION * DEG2RAD) - 1),
            focalLen, prncPt, Point(pts[i].x, pts[i].y));
        double line_x = linePos.dist * sin(linePos.theta * M_PI / 180);
        double line_y = linePos.dist * cos(linePos.theta * M_PI / 180);
        double line_absx = line_x * cos((-1) * pan_tilt.ptpos.PAN_POSITION) -
                           line_y * sin((-1) * pan_tilt.ptpos.PAN_POSITION);
        double line_absy = line_x * sin((-1) * pan_tilt.ptpos.PAN_POSITION) +
                           line_y * cos((-1) * pan_tilt.ptpos.PAN_POSITION);

        // 일정 거리 이상에 존재하는 특징점은 예외처리
        int remove_space_dis = 3000;

        if (linePos.dist < remove_space_dis) {
          // 로컬 노드에 특징점의 데이터를 보내기 위해 데이터 PUBLISH
          vision_feature_Msg.confidence.push_back(condis[i].x);
          vision_feature_Msg.distance.push_back(linePos.dist);
          vision_feature_Msg.point_vec_x.push_back(line_absx);
          vision_feature_Msg.point_vec_y.push_back(line_absy);
        }
      }
      pts.clear();
      condis.clear();
      if (cam_nice_point == 1) {
        publish_localization_msg();
      } else {
        vision_feature_Msg.confidence.clear();
        vision_feature_Msg.distance.clear();
        vision_feature_Msg.point_vec_x.clear();
        vision_feature_Msg.point_vec_y.clear();
      }
    }

    if (cnt_robot != 0)  // 로봇(obstacle)이 카운트 됐을 경우 실행
    {
      vector<Point2f> robot_pts;
      Rect result(0, 0, 0, 0);
      for (std::vector<YOLOV4::YOLO_RESULT>::size_type i = 0;
           i < yolov4.vector_yolo.size(); i++)  // 검출된 객체 수 만큼 반복 실행
      {
        if ((yolov4.vector_yolo[i].NUM == 2))  // 검출된 객체가 로봇일 경우 실행
        {
          // 결과 값에 해당 욜로 데이터를 저장
          result = Rect(yolov4.vector_yolo[i].X, yolov4.vector_yolo[i].Y,
                        yolov4.vector_yolo[i].W, yolov4.vector_yolo[i].H);

          // 버퍼에 로봇의 좌표와 넓이 및 높이 데이터 입력
          char buf[100];
          sprintf(buf, "%d %lf %lf %lf %lf", 2,
                  ((double)result.x + (double)result.width / 2) / 640,
                  ((double)result.y + (double)result.height / 2) / 480,
                  (double)result.width / 640, (double)result.height / 480);
          string value_total(buf);
          data.push_back(value_total);

          // 로봇의 중앙 계산
          double tilt_param =
              result.height / 2 * (1 - (double(result.y) / 480));
          double robot_center_X = result.x + result.width / 2;
          double robot_center_Y = result.y + result.height / 2 + tilt_param;

          // UI 에 로봇의 위치 그리기
          int color = yolov4.vector_yolo[i].SCORE * 255;
          rectangle(result_mat, result, Scalar(color, color, color), 3, 8, 0);
          line(result_mat, Point(robot_center_X, robot_center_Y),
               Point(robot_center_X, robot_center_Y), Scalar(0, 0, 0), 3, 8, 0);

          // 로봇의 좌표 벡터 컨테이너로 저장
          robot_pts.push_back(Point2f(robot_center_X, robot_center_Y));
        }
      }

      // visionMsg의 로봇 벡터 컨테이너 초기화
      visionMsg.robot_vec_x.clear();
      visionMsg.robot_vec_y.clear();

      // 물체와 로봇의 거리 계산
      if (robot_pts.size() != 0) {
        undistortPoints(robot_pts, robot_pts, K_M, D_M, Mat(), NEW_K_M);
      }
      for (std::vector<cv::Point_<float>>::size_type i = 0;
           i < robot_pts.size(); i++) {
        robotPos = calcObjectDistance(
            pan_tilt.ptpos.TILT_POSITION,
            ROBOT_HEIGHT +
                TILT_L * (cos(pan_tilt.ptpos.TILT_POSITION * DEG2RAD) - 1),
            focalLen, prncPt, Point(robot_pts[i].x, robot_pts[i].y));
        double robot_x = robotPos.dist * sin(robotPos.theta * M_PI / 180);
        double robot_y = robotPos.dist * cos(robotPos.theta * M_PI / 180);

        // 로봇(obstacle)의 절대좌표 계산
        robot_absx = robot_x * cos((-1) * pan_tilt.ptpos.PAN_POSITION) -
                     robot_y * sin((-1) * pan_tilt.ptpos.PAN_POSITION);
        robot_absy = robot_x * sin((-1) * pan_tilt.ptpos.PAN_POSITION) +
                     robot_y * cos((-1) * pan_tilt.ptpos.PAN_POSITION);

        // visionMsg의 로봇 벡터 컨테이너에 데이터 저장
        visionMsg.robot_vec_x.push_back(robot_absx);
        visionMsg.robot_vec_y.push_back(robot_absy);
      }
      robot_pts.clear();
    } else  // 로봇(obstacle)이 카운트 되지 않았을 경우 실행
    {
      visionMsg.robot_vec_x.clear();
      visionMsg.robot_vec_y.clear();
      robot_absx = 0;
      robot_absy = 0;
    }
  }

  // 입출력스트림을 통해 이미지를 저장하는 코드, 현재는 사용 안됨
  //  if(save_img_flag)
  //  {
  //      save_img_flag = 0;
  //      save_img_count += 1;
  //      char buf[256];
  //      sprintf(buf,
  //      "/home/robit/catkin_ws/src/Vision-Local/robocup_vision23_yolov4/img/%s%06d.png",
  //      &str_list, save_img_count); imwrite(buf, img); sprintf(buf,
  //      "/home/robit/catkin_ws/src/Vision-Local/robocup_vision23_yolov4/img/%s%06d.txt",
  //      &str_list, save_img_count); ofstream file(buf); if(file.is_open())
  //      {
  //          for(int j = 0; j < data.size(); j++)
  //          {
  //              file << data[j] << endl;
  //          }
  //          file.close();
  //      }
  //      cout<<"save img!!!"<<endl;
  //  }

  return result_mat;
}

void robocup_vision25::publish_localization_msg() {
  // PRE CONDITION : vision_feature_Msg
  // POST CONDITION : vision_feature_Msg
  // PURPOSE : 로컬 노드에 특징점의 데이터를 보내기 위해 데이터 PUBLISH

  vision_feature_Pub->publish(vision_feature_Msg);

  vision_feature_Msg.confidence.clear();
  vision_feature_Msg.distance.clear();
  vision_feature_Msg.point_vec_x.clear();
  vision_feature_Msg.point_vec_y.clear();
}

void robocup_vision25::publish_vision_msg() {
  // PRE CONDITION : visionMsg
  // POST CONDITION : visionMsg
  // PURPOSE : 로컬 노드에 로봇의 데이터를 보내기 위해 데이터 PUBLISH

  visionMsg.ball_cam_x = ball_cam_X;
  visionMsg.ball_cam_y = ball_cam_Y;
  if (ballPos.dist == 0) {
    visionMsg.ball_2d_x = 0;
    visionMsg.ball_2d_y = 0;
  } else {
    visionMsg.ball_2d_x = final_filter_x;
    visionMsg.ball_2d_y = final_filter_y;
  }
  visionMsg.ball_d = ballPos.dist;
  visionMsg.pan = pan_tilt.ptpos.PAN_POSITION;
  visionMsg.tilt = pan_tilt.ptpos.TILT_POSITION;
  visionMsg.ball_speed_x = ball_speed_vec_x;
  visionMsg.ball_speed_y = ball_speed_vec_y;
  visionMsg.ball_speed_level = ball_speed_level;
  visionMsg.scan_mode = scan_value;

  visionPub->publish(visionMsg);

  visionMsg.robot_vec_x.clear();
  visionMsg.robot_vec_y.clear();
}

void robocup_vision25::publish_motor_msg() {
  // PRE CONDITION : pan_tilt
  // POST CONDITION : pan_tilt
  // PURPOSE : 로봇의 팬틸트 모터를 제어하기 위한 데이터 PUBLISH

  dynamixel_rdk_msgs::msg::DynamixelMsgs msg;
  msg.goal_position = pan_tilt.ptpos.PAN_POSITION;
  msg.profile_velocity = 0;
  msg.profile_acceleration = 0;

  Motor_Pub->publish(msg);
}

void robocup_vision25::master_callback(
    const humanoid_interfaces::msg::Master2vision25::SharedPtr msg) {
  if (ocam_error) {
    scan_value = 99;
  } else {
    scan_value = msg->scanmode;
  }

  if (scan_value == 4) {
    pan_tilt.ptpos.PAN_POSITION = msg->pan;
    pan_tilt.ptpos.TILT_POSITION = TILT_D;
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<robocup_vision25>());

  rclcpp::shutdown();
  return 0;
}
