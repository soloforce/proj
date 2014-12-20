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
    mat.setTo(cv::Scalar(255,255,255));
}

void Canvas::drawCircle(cv::Point2f pt, int radius, cv::Scalar color, int width)
{
    if(mat.empty()) return;
#ifdef CANVAS_AA
    cv::circle(mat, pt, radius, color, width, cv::LINE_AA);
#else
    cv::circle(mat, pt, radius, color, width);
#endif
}
void Canvas::drawLine(cv::Point2f pt1, cv::Point2f pt2, cv::Scalar color, int width)
{
    if(mat.empty()) return;
#ifdef CANVAS_AA
    cv::line(mat, pt1, pt2, color, width, cv::LINE_AA);
#else
    cv::line(mat, pt1, pt2, color, width);
#endif
}

void Canvas::drawOriginalPoints(std::vector<BZPoint2f>& pts, cv::Scalar color, int width)
{
    if(pts.empty()) return;
    int n=pts.size();
    if(n<1) return;
    if(n==1){
#ifdef CANVAS_AA
        cv::circle(mat, pts[0], width/2.0 , color, -1, cv::LINE_AA);
#else
        cv::circle(mat, pts[0], width/2.0 , color, -1 );
#endif

    }else{
        for(int i=0; i<n-1; i++){
#ifdef CANVAS_AA
            cv::line(mat, pts[i], pts[i+1], color, width, cv::LINE_AA);
#else
            cv::line(mat, pts[i], pts[i+1], color, width);
#endif
        }
    }
}

void Canvas::drawInterpolatedPoints(std::vector<BZPoint2f>& pts, cv::Scalar color, int width, bool controlPointsVisible)
{
    if(pts.empty()) return;
    const cv::Scalar blue(255,0,0);
    int n=pts.size();

#ifdef CANVAS_AA
    if(n==1){
        cv::circle(mat, pts[0], width/2.0, color, -1, cv::LINE_AA);
        return;
    }else if(n==2){
        cv::line(mat, pts[0], pts[1], color, width, cv::LINE_AA);
        return;
    }

    for(int i=0; i<n-1; i++){

        // draw control points
        if(controlPointsVisible && i>0){
            cv::circle(mat, pts[i].cpt[0], 3, blue, 1);
            cv::circle(mat, pts[i].cpt[1], 3, blue, 1);
            cv::line(mat, pts[i].cpt[0], pts[i].cpt[1], blue, 1);
        }

        cv::Point2f prev=pts[i];
        std::vector<cv::Point2f>& intpts=pts[i].interpolatedPoints;
        int m=intpts.size();
        // draw the lines formed by interpolated points
        for(int j=0; j<m; j++){
            cv::line(mat, prev, intpts[j], color, width, cv::LINE_AA);
            prev=intpts[j];
        }
        cv::line(mat, prev, pts[i+1], color, width, cv::LINE_AA);
    }

    // if it's a loop curve: draw the control points of first & last original points
    if( controlPointsVisible && (pts[0]==pts[n-1]) ){
        cv::circle(mat, pts[0].cpt[0], 3, blue, 1);
        cv::circle(mat, pts[0].cpt[1], 3, blue, 1);
        cv::line(mat, pts[0].cpt[0], pts[0].cpt[1],blue, 1);
    }
#else
    if(n==1){
        cv::circle(mat, pts[0], width/2.0, color, -1);
        return;
    }else if(n==2){
        cv::line(mat, pts[0], pts[1], color, width);
        return;
    }

    for(int i=0; i<n-1; i++){

        // draw control points
        if(controlPointsVisible && i>0){
            cv::circle(mat, pts[i].cpt[0], 3, blue, 1);
            cv::circle(mat, pts[i].cpt[1], 3, blue, 1);
            cv::line(mat, pts[i].cpt[0], pts[i].cpt[1], blue, 1);
        }

        cv::Point2f prev=pts[i];
        std::vector<cv::Point2f>& intpts=pts[i].interpolatedPoints;
        int m=intpts.size();
        // draw the lines formed by interpolated points
        for(int j=0; j<m; j++){
            cv::line(mat, prev, intpts[j], color, width);
            prev=intpts[j];
        }
        cv::line(mat, prev, pts[i+1], color, width);
    }

    // if it's a loop curve: draw the control points of first & last original points
    if( controlPointsVisible && (pts[0]==pts[n-1]) ){
        cv::circle(mat, pts[0].cpt[0], 3, blue, 1);
        cv::circle(mat, pts[0].cpt[1], 3, blue, 1);
        cv::line(mat, pts[0].cpt[0], pts[0].cpt[1],blue, 1);
    }
#endif
}
