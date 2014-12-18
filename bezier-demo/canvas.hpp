#ifndef _CANVAS_HPP_
#define _CANVAS_HPP_

#include "opencv2/core.hpp"
#include "bzpoint.hpp"

class Canvas{
public:
    static const int WIDTH=800;
    static const int HEIGHT=800;
public:
    Canvas(int width=WIDTH, int height=HEIGHT);
    ~Canvas();
public:
    void setMat(cv::Mat& mat);
    cv::Mat& getMat() { return mat; }
    void clear();

    void drawCircle(cv::Point2f pt, int radius, cv::Scalar color, int width);
    void drawLine(cv::Point2f pt1, cv::Point2f pt2, cv::Scalar color, int width);
    void drawOriginalPoints(vector<BZPoint2f>& pts, cv::Scalar color, int width);
    void drawInterpolatedPoints(vector<BZPoint2f>& pts, cv::Scalar color, int width, bool controlPointsVisible=false);

protected:
    cv::Mat mat;
};


#endif
