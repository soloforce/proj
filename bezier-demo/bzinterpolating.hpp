#ifndef _BZINTERPOLATING_HPP_
#define _BZINTERPOLATING_HPP_

#include "bzpoint.hpp"
#include "canvas.hpp"

class BezierInterpolating{
    friend class Canvas;
    static const int DEFAULT_INTERPOLATING_POINTS=500;
public:
    void calcAllControlPoints();
    void calcAllBezierPoints();
    bool empty() const { return pts.empty(); }
    void push_back( Point2f& pt) { pts.push_back(pt); }
    Point2f operator[](int n){ return pts[n]; }
    void clear(){ pts.clear(); }
    int size(){ return pts.size(); }
    void setControlPointsVisibility(bool flag){ controlPointsVisible=flag; }
    Canvas& getCanvas(){ return canvas; }
    Canvas& createCanvas(int width, int height){ canvas=Canvas(width, height); return canvas; }
    vector<BZPoint2f>& getBezierPoints() { return pts; }
protected:
    float getDistance(Point2f& pt1, Point2f& pt2);
    Point2f getMiddlePoint(Point2f& pt1, Point2f& pt2);
    void calcControlPoints(BZPoint2f& prevPt, BZPoint2f& pt, BZPoint2f& nextPt);
    Point2f calcPointOnCubicBezier( BZPoint2f& pt1, BZPoint2f& pt2, float t );
    Point2f calcPointOnQuadraticBezier( BZPoint2f& pt1, BZPoint2f& pt2, Point2f cpt, float t);
    void calcQuadraticBezierPoints(BZPoint2f& pt1, BZPoint2f& pt2, Point2f cpt);
    void calcCubicBezierPoints(BZPoint2f& pt1, BZPoint2f& pt2);
protected:
    Canvas canvas;
    vector<BZPoint2f> pts;
    bool controlPointsVisible;
};

#endif
