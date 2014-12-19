#include "bzinterpolating.h"

float BezierInterpolating::getDistance(cv::Point2f& pt1, cv::Point2f& pt2){
    float f1=pt2.x-pt1.x;
    float f2=pt2.y-pt1.y;
    return sqrt(f1*f1+f2*f2);
}

cv::Point2f BezierInterpolating::getMiddlePoint(cv::Point2f& pt1, cv::Point2f& pt2){
    cv::Point2f pt;
    pt.x=(pt1.x+pt2.x)/2.0;
    pt.y=(pt1.y+pt2.y)/2.0;
}

void BezierInterpolating::calcControlPoints(BZPoint2f& prevPt, BZPoint2f& pt, BZPoint2f& nextPt)
{
    cv::Point2f c1, c2;

    float len1=getDistance(prevPt, pt);
    float len2=getDistance(pt, nextPt);
    float k1=len1/(len1+len2);
    float k2=1-k1;

    // Two choice here: smoothFactor or globalSmoothFactor
    //k1=pt.smoothFactor*k1;
    //k2=pt.smoothFactor*k2;
    k1=BZPoint2f::globalSmoothFactor*k1;
    k2=BZPoint2f::globalSmoothFactor*k2;

    // middle points between original points
    cv::Point2f mpt1((prevPt.x+pt.x)/2.0, (prevPt.y+pt.y)/2.0);
    cv::Point2f mpt2((pt.x+nextPt.x)/2.0, (pt.y+nextPt.y)/2.0);

    // control points
    pt.cpt[0]=cv::Point2f(pt.x+(mpt1.x-mpt2.x)*k1, pt.y+(mpt1.y-mpt2.y)*k1);
    pt.cpt[1]=cv::Point2f(pt.x+(mpt2.x-mpt1.x)*k2, pt.y+(mpt2.y-mpt1.y)*k2);
}



void BezierInterpolating::calcAllControlPoints()
{
    int n=pts.size();
    if(n < 3) return;

    for(int i=1; i<n-1; i++){
        calcControlPoints(pts[i-1], pts[i], pts[i+1]);
    }

    // handle the first & last original points
    const int min_dist=5;

    if( getDistance(pts[0],pts[n-1]) < min_dist){
        // When the first & last points are quite close, consider them
        // have the same coordinates, such that the interpolated curve
        // is a loop curve.

        // make coordinates the same.
        pts[0].x=(pts[0].x+pts[n-1].x)/2.0;
        pts[n-1]=pts[0];

        calcControlPoints(pts[n-2], pts[0], pts[1]);
        calcControlPoints(pts[n-2], pts[n-1], pts[1]);
    }else{
        pts[0].cpt[0]=pts[0].cpt[1]=pts[1].cpt[0];
        pts[n-1].cpt[0]=pts[n-1].cpt[1]=pts[n-2].cpt[1];
    }

}


/** 从四个点（从左至右，中间两个为控制点）,根据三次贝塞尔曲线方程计算一个插值点
 pt1為起始點;
 pt1.cpt[1]為第一個控制點;
 pt2.cpt[0]為第二個控制點;
 pt2 為結束點，或上圖中的P3;
 t為參數值，0 <= t <= 1
 Ref: http://zh.wikipedia.org/wiki/%E8%B2%9D%E8%8C%B2%E6%9B%B2%E7%B7%9A
*/
cv::Point2f BezierInterpolating::calcPointOnCubicBezier( BZPoint2f& pt1, BZPoint2f& pt2, float t )
{
    float   ax, bx, cx;
    float   ay, by, cy;
    float   tSquared, tCubed;
    cv::Point2f result;

    /*計算多項式係數*/
    cx = 3.0 * (pt1.cpt[1].x -pt1.x);
    bx = 3.0 * (pt2.cpt[0].x -  pt1.cpt[1].x) - cx;
    ax = pt2.x - pt1.x - cx - bx;

    cy = 3.0 * (pt1.cpt[1].y - pt1.y);
    by = 3.0 * (pt2.cpt[0].y - pt1.cpt[1].y) - cy;
    ay = pt2.y - pt1.y - cy - by;

    /*計算位於參數值t的曲線點*/

    tSquared = t * t;
    tCubed = tSquared * t;

    result.x = (ax * tCubed) + (bx * tSquared) + (cx * t) + pt1.x;
    result.y = (ay * tCubed) + (by * tSquared) + (cy * t) + pt1.y;

    return result;
}


//! cpt: control point between pt1 & pt2
cv::Point2f BezierInterpolating::calcPointOnQuadraticBezier( BZPoint2f& pt1, BZPoint2f& pt2, cv::Point2f cpt, float t)
{
    cv::Point2f result;
    float t2 = t * t;
    result.x=(1 + t2 -2*t) * pt1.x + 2*t*(1-t)*cpt.x + t2*pt2.x;
    result.y=(1 + t2 -2*t) * pt1.y + 2*t*(1-t)*cpt.y + t2*pt2.y;
    return result;
}


void BezierInterpolating::calcCubicBezierPoints(BZPoint2f& pt1, BZPoint2f& pt2)
{
    float dt;
    int i;

    if(pt1.interpolated){
         pt1.interpolatedPoints.clear();
    }

    /*
    if(pt1.pointsToBeInterpolated < 1) {
        return;
    }
    int n=pt1.pointsToBeInterpolated ;
    */

    int n=DEFAULT_INTERPOLATING_POINTS;

    dt = 1.0 / ( n + 1 );
    for( i = 0; i < n; i++){
        pt1.interpolatedPoints.push_back(calcPointOnCubicBezier(pt1, pt2, (i+1)*dt ) );
    }

    pt1.interpolated=true;
}

void BezierInterpolating::calcQuadraticBezierPoints(BZPoint2f& pt1, BZPoint2f& pt2, cv::Point2f cpt){
    float dt;
    int i;

    if(pt1.interpolated){
         pt1.interpolatedPoints.clear();
    }

    /*
    if(pt1.pointsToBeInterpolated < 1) {
        return;
    }
    int n=pt1.pointsToBeInterpolated ;
    */

    int n=DEFAULT_INTERPOLATING_POINTS;

    dt = 1.0 / ( n + 1 );
    for( i = 0; i < n; i++){
        pt1.interpolatedPoints.push_back(calcPointOnQuadraticBezier(pt1, pt2, cpt, (i+1)*dt ) );
    }

    pt1.interpolated=true;
}

void BezierInterpolating::calcAllBezierPoints()
{
    int n=pts.size();
    if(n < 3) return;
    if(n==3){
        calcQuadraticBezierPoints(pts[0], pts[1], pts[1].cpt[0]);
        calcQuadraticBezierPoints(pts[1], pts[2], pts[1].cpt[1]);
        return;
    }

    for(int i=1; i<n-2; i++){
        calcCubicBezierPoints(pts[i], pts[i+1]);
    }

    if( pts[0] == pts[n-1] ){
        calcCubicBezierPoints(pts[0], pts[1]);
        calcCubicBezierPoints(pts[n-2], pts[n-1]);
    }else{
        // the curve between first two original points
        calcQuadraticBezierPoints(pts[0], pts[1], pts[1].cpt[0]);
        // the curve between last two original points
        calcQuadraticBezierPoints(pts[n-2], pts[n-1], pts[n-2].cpt[1]);
    }
}

