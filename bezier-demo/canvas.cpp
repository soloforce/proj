#include "canvas.hpp"

Canvas::Canvas(int width, int height)
{
    mat=cv::Mat(height, width, CV_8UC3);
    clear();
}

Canvas::~Canvas()
{

}

void Canvas::setMat(cv::Mat& m)
{
    mat=m;
}

void Canvas::clear()
{
    if(mat.empty()) return;
    mat.setTo(Scalar(255,255,255));
}

void Canvas::drawCircle(cv::Point2f pt, int radius, cv::Scalar color, int width)
{
    if(mat.empty()) return;
    cv::circle(mat, pt, radius, color, width, cv::LINE_AA);
}
void Canvas::drawLine(cv::Point2f pt1, cv::Point2f pt2, cv::Scalar color, int width)
{
    if(mat.empty()) return;
    cv::line(mat, pt1, pt2, color, width, cv::LINE_AA);
}

void Canvas::drawOriginalPoints(vector<BZPoint2f>& pts, Scalar color, int width)
{
    int n=pts.size();
    if(n<1) return;
    if(n==1)  circle(mat, pts[0], width/2.0 , color, -1, cv::LINE_AA );
    else{
        for(int i=0; i<n-1; i++){
            cv::line(mat, pts[i], pts[i+1], color, width, cv::LINE_AA);
        }
    }
}

void Canvas::drawInterpolatedPoints(vector<BZPoint2f>& pts, cv::Scalar color, int width, bool controlPointsVisible)
{
    if(pts.empty()) return;
    int n=pts.size();
    if(n==1){
        circle(mat, pts[0], width/2.0, color, -1, cv::LINE_AA);
        return;
    }else if(n==2){
        line(mat, pts[0], pts[1], color, width, cv::LINE_AA);
        return;
    }

    for(int i=0; i<n-1; i++){
        if(controlPointsVisible && i>0){
            cv::circle(mat, pts[i].cpt[0], 3, cv::Scalar(0,255,0), 1, cv::LINE_AA);
            cv::circle(mat, pts[i].cpt[1], 3, cv::Scalar(0,255,0), 1, cv::LINE_AA);
            cv::line(mat, pts[i].cpt[0], pts[i].cpt[1], cv::Scalar(0,255,0), 1, cv::LINE_AA);
        }

        cv::Point2f prev=pts[i];
        vector<cv::Point2f>& intpts=pts[i].interpolatedPoints;
        int n=intpts.size();
        for(int j=0; j<n; j++){
            cv::line(mat, prev, intpts[j], color, width, cv::LINE_AA);
            prev=intpts[j];
        }
        cv::line(mat, prev, pts[i+1], color, width, cv::LINE_AA);
    }
}
