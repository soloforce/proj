#ifndef _CANVAS_H_
#define _CANVAS_H_

#include <opencv2/opencv.hpp>
#include <vector>

#include "bzpoint2f.h"
#include "widget.h"

/** Pen class.
*   Details:
*/
class Pen{
public:
    static const int LARGE_THICKNESS=8;
    static const int MEDIUM_THICKNESS=4;
    static const int SMALL_THICKNESS=1;
public:
    Pen():color(cv::Scalar(255,255,255)), thickness(1) { }
    Pen(cv::Scalar c, int w=1, int t=CV_AA):color(c),thickness(w), type(t){}
public:
    cv::Scalar color;
    int thickness;
    int type;
};

/** Canvas class.
*   Details:
*/
class Canvas{
public:
    Canvas();
    virtual ~Canvas();
public:
    void setMat(cv::Mat& m);
    void setValidRegion(cv::Rect rect);
    void drawBorder(Pen pen);
    void reset();
    void clear();
    void drawCircle(cv::Point2f center, int radius, Pen pen);
    void drawRectangle(cv::Rect rect, Pen pen);
    void drawLine(cv::Point2f prevPt, cv::Point2f pt, Pen pen);
    void drawWidgets();
    void drawSelectedPen();
    bool selectPen(cv::Point2d pt);
    void drawOriginalPoints(std::vector<BZPoint2f>& pts, Pen& pen);
    void drawInterpolatedPoints(std::vector<BZPoint2f>& pts, Pen& pen);
    void initWidgets();
public:
    Pen selectedPen;
    cv::Mat mat;
    cv::Rect validRegion;
    std::vector<Widget*> widgets;
    cv::Scalar bgColor;
    bool interpEnabled;
};


#endif