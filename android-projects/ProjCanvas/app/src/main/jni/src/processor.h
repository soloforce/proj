#ifndef _PROCESSOR_H_
#define _PROCESSOR_H_

#include "canvas.h"
#include "calibrator.h"
#include "state.h"
#include "bzinterpolating.h"

class Processor{
public:
    static const int MAX_TOUCHED_POINTS=1;
public:
    Processor(){
        inited=false;
    }
    void process(cv::Mat& frame);
    void init(cv::Mat& result, cv::Rect rectCamera, cv::Rect rectCanvas);
    bool isInitialized() const { return inited; }
    void reset();
protected:
    void getTouchedPoints(cv::Mat& frame, std::vector<cv::Point2d>& touchedPoints);
    void setValidRegion(cv::Rect rect){ validRegion=rect; }
    void drawAction();
protected:
    cv::Rect validRegion;
    Canvas canvas;
    Calibrator calib;
    State state;
    BezierInterpolating bzInterp;
private:
    bool inited;
};

#endif