#ifndef _BZINTERPOLATING_HPP_
#define _BZINTERPOLATING_HPP_

#include "bzpoint.hpp"


class BezierInterpolating{
    friend class Canvas;
    static const int DEFAULT_INTERPOLATING_POINTS=30;
public:
    void calcAllControlPoints();
    void calcAllBezierPoints();
    bool empty() const { return pts.empty(); }
    void push_back( cv::Point2f& pt) { pts.push_back(pt); }
    cv::Point2f operator[](int n){ return pts[n]; }
    void clear(){ pts.clear(); }
    int size(){ return pts.size(); }
    void setControlPointsVisibility(bool flag){ controlPointsVisible=flag; }
    std::vector<BZPoint2f>& getBezierPoints() { return pts; }
protected:
    float getDistance(cv::Point2f& pt1, cv::Point2f& pt2);
    cv::Point2f getMiddlePoint(cv::Point2f& pt1, cv::Point2f& pt2);
    void calcControlPoints(BZPoint2f& prevPt, BZPoint2f& pt, BZPoint2f& nextPt);
    cv::Point2f calcPointOnCubicBezier( BZPoint2f& pt1, BZPoint2f& pt2, float t );
    cv::Point2f calcPointOnQuadraticBezier( BZPoint2f& pt1, BZPoint2f& pt2, cv::Point2f cpt, float t);
    void calcQuadraticBezierPoints(BZPoint2f& pt1, BZPoint2f& pt2, cv::Point2f cpt);
    void calcCubicBezierPoints(BZPoint2f& pt1, BZPoint2f& pt2);
protected:
    std::vector<BZPoint2f> pts;
    bool controlPointsVisible;
};

#endif
