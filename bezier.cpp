#include <iostream>
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;
const int CANVAS_WIDTH=1200;
const int CANVAS_HEIGHT=800;

//! bezier cubic curve
class bzPoint2f : public cv::Point2f{
    friend class bezierInterpolating;
    static const float DEFAULT_SMOOTH_FACTOR=0.6;
public:
    bzPoint2f(){
        x=y=-1;
        cpt[0]=cpt[1]=Point2f(-1,-1);
        smoothFactor=DEFAULT_SMOOTH_FACTOR;
        interpolated=false;
    }

    bzPoint2f(int a, int b){
        x=a;
        y=b;
        cpt[0]=cpt[1]=Point2f(-1,-1);
        smoothFactor=DEFAULT_SMOOTH_FACTOR;
        interpolated=false;
    }

    bzPoint2f(const bzPoint2f& bpt){
        x=bpt.x;
        y=bpt.y;
        cpt[0]=bpt.cpt[0];
        cpt[1]=bpt.cpt[1];
        smoothFactor=bpt.smoothFactor;
        interpolated=false;
    }

    bzPoint2f(const Point2f& pt){
        x=pt.x;
        y=pt.y;
        smoothFactor=DEFAULT_SMOOTH_FACTOR;
        interpolated=false;
    }


    bzPoint2f& operator=(const bzPoint2f& bpt){
        x=bpt.x;
        y=bpt.y;
        cpt[0]=bpt.cpt[0];
        cpt[1]=bpt.cpt[1];
        smoothFactor=bpt.smoothFactor;
        interpolated=false;
    }


    bzPoint2f& operator=(const Point2f& pt){
        x=pt.x;
        y=pt.y;
        smoothFactor=DEFAULT_SMOOTH_FACTOR;
        interpolated=false;
    }

    virtual ~bzPoint2f(){};
public:
    float getSmoothFactor() const { return smoothFactor; }
    void setSmoothFactor(float sm) {
        smoothFactor=sm;
        if(smoothFactor<0.0) smoothFactor=0.0;
        if(smoothFactor>1.0) smoothFactor=1.0;
    }
protected:
    // control points:
    // 1. There are 2 control points associated with a given original
    // point for cubic bezier interpolation, one before, one after.
    // 2. There is only one control point for quadratic bezier interpolation.
    Point2f cpt[2];


    float smoothFactor; // 0~1, the bigger the smoother

    vector<Point2f> interpolatedPoints;
    bool interpolated;
    int pointsToBeInterpolated;
};



class bezierInterpolating{
public:
    float getDistance(Point2f& pt1, Point2f& pt2){
        float f1=pt2.x-pt1.x;
        float f2=pt2.y-pt1.y;
        return sqrt(f1*f1+f2*f2);
    }

    Point2f getMiddlePoint(Point2f& pt1, Point2f& pt2){
        Point2f pt;
        pt.x=(pt1.x+pt2.x)/2.0;
        pt.y=(pt1.y+pt2.y)/2.0;
    }
    void calcAllControlPoints();
    void calcControlPoints(bzPoint2f& prevPt, bzPoint2f& pt, bzPoint2f& nextPt)
    {
        Point2f c1, c2;

        float len1=getDistance(prevPt, pt);
        float len2=getDistance(pt, nextPt);
        float k1=len1/(len1+len2);
        float k2=1-k1;
        k1=pt.smoothFactor*k1;
        k2=pt.smoothFactor*k2;

        // middle points between original points
        Point2f mpt1((prevPt.x+pt.x)/2.0, (prevPt.y+pt.y)/2.0);
        Point2f mpt2((pt.x+nextPt.x)/2.0, (pt.y+nextPt.y)/2.0);

        // control points
        pt.cpt[0]=Point2f(pt.x+(mpt1.x-mpt2.x)*k1, pt.y+(mpt1.y-mpt2.y)*k1);
        pt.cpt[1]=Point2f(pt.x+(mpt2.x-mpt1.x)*k2, pt.y+(mpt2.y-mpt1.y)*k2);
    }

    Point2f calcPointOnCubicBezier( bzPoint2f& pt1, bzPoint2f& pt2, float t );
    Point2f calcPointOnQuadraticBezier( bzPoint2f& pt1, bzPoint2f& pt2, Point2f cpt, float t);
    void calcQuadraticBezierPoints(bzPoint2f& pt1, bzPoint2f& pt2, Point2f cpt);
    void calcCubicBezierPoints(bzPoint2f& pt1, bzPoint2f& pt2);
    void calcAllBezierPoints();
    void drawInterpolatedPoints(Mat& canvas, Scalar color, int width);
    bool empty() const { return pts.empty(); }
    void push_back( Point2f& pt) { pts.push_back(pt); }
    Point2f operator[](int n){ return pts[n]; }
    void clear(){ pts.clear(); }
    int size(){ return pts.size(); }
    void setControlPointsVisibility(bool flag){ controlPointsVisible=flag; }
    void drawOriginalPoints(Mat&, Scalar, int);
protected:
    vector<bzPoint2f> pts;
    bool controlPointsVisible;
};

void bezierInterpolating::calcAllControlPoints()
{
    if(pts.size() < 3) return;

    for(int i=1; i<pts.size()-1; i++){
        calcControlPoints(pts[i-1], pts[i], pts[i+1]);
    }

    bzPoint2f& firstPt=pts[0];
    bzPoint2f& lastPt=pts[pts.size()-1];

    if( (firstPt.x==lastPt.x) && (firstPt.y==lastPt.y) ){
        calcControlPoints(pts[pts.size()-2], firstPt, pts[1]);
        lastPt=firstPt; // set the last point the same as first one.
    }else{
        firstPt.cpt[0]=firstPt.cpt[1]=pts[1].cpt[0];
        lastPt.cpt[0]=lastPt.cpt[1]=pts[pts.size()-2].cpt[1];
    }

}

void bezierInterpolating::drawOriginalPoints(Mat& canvas, Scalar color, int width)
{
    int n=pts.size();
    if(n<1) return;
    if(n==1)  circle(canvas, pts[0], width/2.0 , color, -1, LINE_AA );
    else{
        for(int i=0; i<n-1; i++){
            line(canvas, pts[i], pts[i+1], color, width, LINE_AA);
        }
    }
}


/*
 cp在此是四個元素的陣列:
 cp[0]為起始點，或上圖中的P0: pt1
 cp[1]為第一個控制點，或上圖中的P1: pt1.cpt[1]
 cp[2]為第二個控制點，或上圖中的P2: pt2.cpt[0]
 cp[3]為結束點，或上圖中的P3 : pt2
 t為參數值，0 <= t <= 1
*/

Point2f bezierInterpolating::calcPointOnCubicBezier( bzPoint2f& pt1, bzPoint2f& pt2, float t )
{
    float   ax, bx, cx;
    float   ay, by, cy;
    float   tSquared, tCubed;
    Point2f result;

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
Point2f bezierInterpolating::calcPointOnQuadraticBezier( bzPoint2f& pt1, bzPoint2f& pt2, Point2f cpt, float t)
{
    Point2f result;
    float t2 = t * t;
    result.x=(1 + t2 -2*t) * pt1.x + 2*t*(1-t)*cpt.x + t2*pt2.x;
    result.y=(1 + t2 -2*t) * pt1.y + 2*t*(1-t)*cpt.y + t2*pt2.y;
    return result;
}

/*
 ComputeBezier以控制點cp所產生的曲線點，填入Point2D結構的陣列。
 呼叫者必須分配足夠的記憶體以供輸出結果，其為<sizeof(Point2D) numberOfPoints>
*/

void bezierInterpolating::calcCubicBezierPoints(bzPoint2f& pt1, bzPoint2f& pt2)
{
    float   dt;
    int    i;

    if(pt1.interpolated){
         pt1.interpolatedPoints.clear();
    }

    /*
    if(pt1.pointsToBeInterpolated < 1) {
        return;
    }
    int n=pt1.pointsToBeInterpolated ;
    */

    int n=250;

    dt = 1.0 / ( n + 1 );
    for( i = 0; i < n; i++){
        pt1.interpolatedPoints.push_back(calcPointOnCubicBezier(pt1, pt2, (i+1)*dt ) );
    }

    pt1.interpolated=true;
}

void bezierInterpolating::calcQuadraticBezierPoints(bzPoint2f& pt1, bzPoint2f& pt2, Point2f cpt){
    float   dt;
    int    i;

    if(pt1.interpolated){
         pt1.interpolatedPoints.clear();
    }

    /*
    if(pt1.pointsToBeInterpolated < 1) {
        return;
    }
    int n=pt1.pointsToBeInterpolated ;
    */

    int n=250;

    dt = 1.0 / ( n + 1 );
    for( i = 0; i < n; i++){
        pt1.interpolatedPoints.push_back(calcPointOnQuadraticBezier(pt1, pt2, cpt, (i+1)*dt ) );
    }

    pt1.interpolated=true;
}

void bezierInterpolating::calcAllBezierPoints()
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

    // the curve between first two original points
    calcQuadraticBezierPoints(pts[0], pts[1], pts[1].cpt[0]);

    // the curve between last two original points
    calcQuadraticBezierPoints(pts[n-2], pts[n-1], pts[n-2].cpt[1]);
}

void bezierInterpolating::drawInterpolatedPoints(Mat& canvas, Scalar color, int width)
{
    if(pts.empty()) return;
    int n=pts.size();
    if(n==1){
        circle(canvas, pts[0], width/2.0, color, -1, LINE_AA);
        return;
    }else if(n==2){
        line(canvas, pts[0], pts[1], color, width, LINE_AA);
        return;
    }

    for(int i=0; i<n-1; i++){
        if(controlPointsVisible && i>0){
            circle(canvas, pts[i].cpt[0], 3, Scalar(0,255,0), 1, LINE_AA);
            circle(canvas, pts[i].cpt[1], 3, Scalar(0,255,0), 1, LINE_AA);
            line(canvas, pts[i].cpt[0], pts[i].cpt[1], Scalar(0,255,0), 1, LINE_AA);
        }

        Point2f prev=pts[i];
        vector<Point2f>& intpts=pts[i].interpolatedPoints;
        int n=intpts.size();
        for(int j=0; j<n; j++){
            line(canvas, prev, intpts[j], color, width, LINE_AA);
            prev=intpts[j];
        }
        line(canvas, prev, pts[i+1], color, width, LINE_AA);
    }

}

bezierInterpolating bzInterp;





void onMouse(int event, int x, int y, int, void* extraData)
{


    Mat& canvas=*((Mat*)extraData);
    if( event!=EVENT_LBUTTONDOWN) return;
    bzPoint2f cur(x,y);
    if(bzInterp.empty()){
        circle(canvas, cur, 1 , Scalar(0,0,0), -1, LINE_AA );
    }else{
        bzPoint2f pre= bzInterp[bzInterp.size()-1];
        line(canvas, (Point2f)pre, (Point2f)cur,  Scalar(0,0,0), 1, LINE_AA);
    }
    bzInterp.push_back(cur);

}

int main()
{
    Mat canvas(CANVAS_HEIGHT, CANVAS_WIDTH, CV_8UC3);
    canvas.setTo(Scalar(255,255,255));
    namedWindow("Canvas", 0);
    setMouseCallback("Canvas", onMouse, &canvas);
    bool isRunning=true;
    bool isControlPointsVisible=false;
    while(isRunning){
        imshow("Canvas", canvas);
        int c=waitKey(30);

        switch( c&255 ){
        case 27: //ESC key pressed
            cout<<"Exiting ..."<<endl;
            isRunning=false;
            break;
        case 'b':
            bzInterp.calcAllControlPoints();
            bzInterp.calcAllBezierPoints();
            bzInterp.drawInterpolatedPoints(canvas, Scalar(0,0,255), 1);
            break;
        case 'c':
            canvas.setTo(Scalar(255,255,255));
            bzInterp.clear();
            break;
        case 'v':
            isControlPointsVisible=!isControlPointsVisible;
            bzInterp.setControlPointsVisibility(isControlPointsVisible);
            canvas.setTo(Scalar(255,255,255));
            bzInterp.drawInterpolatedPoints(canvas, Scalar(0,0,255), 1);
            break;
        case 'o':
            bzInterp.drawOriginalPoints(canvas, Scalar(100,100,100), 1);
            break;
        }
    }
    return 0;
}
