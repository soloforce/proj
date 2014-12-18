#ifndef _CALIBRATOR_H_
#define _CALIBRATOR_H_

#include <opencv2/opencv.hpp>
#include <vector>


class Calibrator{
public:
    static const int CALIB_POINTS_COUNT=4;

public:
    Calibrator(){
        isCalibrated=false;
    }

    void calibrate(){
        if( (!cameraPoints.size()) ||
            (!targetPoints.size()) ||
            (cameraPoints.size()!=CALIB_POINTS_COUNT) ||
            (targetPoints.size()!=CALIB_POINTS_COUNT) ) return;

        homo=cv::findHomography(cameraPoints, targetPoints);

        if(!homo.empty()) isCalibrated=true;
    }

    void reset(){
        cameraPoints.clear();
        targetPoints.clear();
        isCalibrated=false;
        homo=cv::Mat();
    }
public:
    std::vector<cv::Point2d> cameraPoints;
    std::vector<cv::Point2d> targetPoints;
    cv::Mat homo;
    bool isCalibrated;

};



#endif