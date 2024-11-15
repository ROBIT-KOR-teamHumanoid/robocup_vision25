#ifndef YOLOV4_HPP
#define YOLOV4_HPP

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

namespace YOLOV4 {

struct YOLO_RESULT {
    int NUM;
    double SCORE;
    int X;
    int Y;
    int W;
    int H;
};

class YoloV4 {
public:
    YoloV4() {
        // For Open Model
        cfg_name = "/home/robit/colcon_ws/src/robocup_vision25/net/yolov4-tiny-1ch-cls3-v7.cfg";
        weight_name = "/home/robit/colcon_ws/src/robocup_vision25/net/yolov4-tiny-1ch-cls3-v7_100000.weights";

        // For Model Param
        NUM_CLASSES = 3;
        CONFIDENCE_THRESHOLD = 0.1;
        SCORE_THRESHOLD_B = 0.8;
        NMS_THRESHOLD_B = 0.8;
        SCORE_THRESHOLD_L = 0.5;
        NMS_THRESHOLD_L = 0.5;
        SCORE_THRESHOLD_R = 0.8;
        NMS_THRESHOLD_R = 0.8;

        // Load the network
        YOLO = cv::dnn::readNetFromDarknet(cfg_name, weight_name);
        YOLO.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        YOLO.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }

    std::vector<YOLO_RESULT> vector_yolo;

    std::vector<YOLO_RESULT> YoloRun(const cv::Mat& img);

private:
    std::string cfg_name;
    std::string weight_name;
    int NUM_CLASSES;
    double CONFIDENCE_THRESHOLD;
    double SCORE_THRESHOLD_B;
    double NMS_THRESHOLD_B;
    double SCORE_THRESHOLD_L;
    double NMS_THRESHOLD_L;
    double SCORE_THRESHOLD_R;
    double NMS_THRESHOLD_R;

    cv::dnn::Net YOLO;
};

std::vector<YOLO_RESULT> YoloV4::YoloRun(const cv::Mat& img) {
    cv::Mat image = img.clone();
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(2);
    clahe->apply(image, image);

    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob, 1/255.0, cv::Size(416, 416), cv::Scalar(), false, false);

    YOLO.setInput(blob);

    std::vector<cv::Mat> outs;
    YOLO.forward(outs, YOLO.getUnconnectedOutLayersNames());

    std::vector<std::vector<int>> indices(NUM_CLASSES);
    std::vector<std::vector<cv::Rect>> boxes(NUM_CLASSES);
    std::vector<std::vector<float>> scores(NUM_CLASSES);

    for (size_t i = 0; i < outs.size(); ++i) {
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
            cv::Mat scoresMat = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point classIdPoint;
            double confidence;
            cv::minMaxLoc(scoresMat, 0, &confidence, 0, &classIdPoint);
            if (confidence > CONFIDENCE_THRESHOLD) {
                int centerX = (int)(data[0] * image.cols);
                int centerY = (int)(data[1] * image.rows);
                int width = (int)(data[2] * image.cols);
                int height = (int)(data[3] * image.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                int classId = classIdPoint.x;
                boxes[classId].push_back(cv::Rect(left, top, width, height));
                scores[classId].push_back((float)confidence);
            }
        }
    }

    std::vector<YOLO_RESULT> results;
    for (int i = 0; i < NUM_CLASSES; ++i) {
        cv::dnn::NMSBoxes(boxes[i], scores[i], SCORE_THRESHOLD_B, NMS_THRESHOLD_B, indices[i]);
        for (size_t j = 0; j < indices[i].size(); ++j) {
            int idx = indices[i][j];
            YOLO_RESULT result;
            result.NUM = i;
            result.SCORE = scores[i][idx];
            result.X = boxes[i][idx].x;
            result.Y = boxes[i][idx].y;
            result.W = boxes[i][idx].width;
            result.H = boxes[i][idx].height;
            results.push_back(result);
        }
    }

    return results;
}

} // namespace YOLOV4

#endif // YOLOV4_HPP
