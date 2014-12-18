#ifndef _BZPOINT_HPP_
#define _BZPOINT_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

//! bezier cubic curve
class BZPoint2f : public cv::Point2f{
    friend class BezierInterpolating;
    friend class Canvas;
    static const float DEFAULT_SMOOTH_FACTOR=0.6;
public:
    BZPoint2f();
    BZPoint2f(int a, int b);
    BZPoint2f(const BZPoint2f& bpt);
    BZPoint2f(const Point2f& pt);
    BZPoint2f& operator=(const BZPoint2f& bpt);
    BZPoint2f& operator=(const Point2f& pt);
    bool operator == (const BZPoint2f& bpt);
    bool operator == (const Point2f& bpt);
    virtual ~BZPoint2f();
public:
    float getSmoothFactor() const;
    void setSmoothFactor(float sm);
protected:
    // control points:
    // 1. There are 2 control points associated with a given original
    // point for cubic bezier interpolation, one before, one after.
    // 2. There is only one control point for quadratic bezier interpolation.
    Point2f cpt[2];

    // smooth factor ranges (0.0~1.0), 0.5 is usually a good choice
    float smoothFactor;

    // vector to store the interpolated points
    vector<Point2f> interpolatedPoints;
    bool interpolated;

    int pointsToBeInterpolated;
};


#endif
