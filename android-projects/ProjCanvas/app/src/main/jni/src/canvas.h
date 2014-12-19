#ifndef _CANVAS_H_
#define _CANVAS_H_

#include <opencv2/opencv.hpp>
#include <vector>

#include "bzpoint2f.h"

/** Pen class.
*   Details:
*/
class Pen{
public:
    Pen():color(cv::Scalar(255,255,255)), thickness(1) { }
    Pen(cv::Scalar c, int w=1, int t=CV_AA):color(c),thickness(w), type(t){}
public:
    cv::Scalar color;
    int thickness;
    int type;
};


class Palette{
public:
	Palette(cv::Rect r, cv::Scalar c, int t):
		rect(r), color(c),thickness(t){}
public:
	cv::Rect rect;
	cv::Scalar color;
	int thickness;
};

/** Canvas class.
*   Details:
*/
class Canvas{
public:
    Canvas(){
        selectedPen=Pen(cv::Scalar(255,255,255), 1);
    }

public:
    void setMat(cv::Mat& m){
        mat=m;
    }

    void setValidRegion(cv::Rect rect){
        validRegion=rect;
    }

    void drawRegion(Pen pen){
        cv::rectangle(mat, validRegion, pen.color, pen.thickness, pen.type);
    }

    void clear(){
        selectedPen=Pen(cv::Scalar(255,255,255), 1);
        mat.setTo(cv::Scalar(0,0,0));
        drawRegion(Pen(cv::Scalar(255,255,255), 1));
        drawCircle(validRegion.tl(), 3, Pen(cv::Scalar(255,0,0), -1));
    }

    void drawCircle(cv::Point center, int radius, Pen pen){
        if(mat.empty()) return;
        cv::circle(mat, center, radius, pen.color, pen.thickness, pen.type);
    }

    void drawRectangle(cv::Rect rect, Pen pen)
    {
        if(mat.empty()) return;
        cv::rectangle(mat, rect, pen.color, pen.thickness, pen.type);
    }

    void drawLine(cv::Point prevPt, cv::Point pt, Pen pen){
        if(mat.empty()) return;
        cv::line(mat, prevPt, pt, pen.color, pen.thickness, pen.type);
    }

    void drawPalettes()
    {
        if(mat.empty()) return;
        for(int i=0; i<palettes.size(); i++)
            drawRectangle( palettes[i].rect, Pen(palettes[i].color,  palettes[i].thickness));
    }

    void drawSelectedColor()
    {
        if(mat.empty()) return;
        cv::Point2d pt(validRegion.br().x-75, validRegion.br().y-75);
        drawCircle(pt, 25, Pen(selectedPen.color, -1));
    }

    void selectPen(cv::Point2d pt)
    {
    	if(palettes.size()) for(int i=0; i<palettes.size(); i++){
    		if(!palettes[i].rect.contains(pt)) continue;
    		if(i==0){
    			clear();
                drawRegion(Pen(cv::Scalar(0,255,0), 1));
                drawPalettes();
    			break;
    		}else selectedPen=Pen(palettes[i].color, 1);
    		drawSelectedColor();
    	}
    }

    void drawOriginalPoints(std::vector<BZPoint2f>& pts, Pen& pen)
    {
        int n=pts.size();
        if(n<1) return;
        if(n==1){
            cv::circle(mat, pts[0], pen.thickness/2.0 , pen.color, -1, CV_AA);
        }else{
            for(int i=0; i<n-1; i++){
                cv::line(mat, pts[i], pts[i+1], pen.color, pen.thickness, CV_AA);
            }
        }
    }

    void drawInterpolatedPoints(std::vector<BZPoint2f>& pts, Pen& pen)
    {
        const cv::Scalar blue(255,0,0);
        if(pts.empty()) return;
        int n=pts.size();

        if(n==1){
            cv::circle(mat, pts[0], pen.thickness/2.0, pen.color, -1, CV_AA);
            return;
        }else if(n==2){
            cv::line(mat, pts[0], pts[1], pen.color, pen.thickness, CV_AA);
            return;
        }

        for(int i=0; i<n-1; i++){
            cv::Point2f prev=pts[i];
            std::vector<cv::Point2f>& intpts=pts[i].interpolatedPoints;
            int m=intpts.size();
            // draw the lines formed by interpolated points
            for(int j=0; j<m; j++){
                cv::line(mat, prev, intpts[j], pen.color, pen.thickness, CV_AA);
                prev=intpts[j];
            }
            cv::line(mat, prev, pts[i+1], pen.color, pen.thickness, CV_AA);
        }
    }

    void initPalettes()
    {
    	const int w=50;
    	int offx=validRegion.br().x-100;
    	int offy=validRegion.tl().y+w;

    	Palette pa_empty(cv::Rect(validRegion.tl().x+w, validRegion.br().y-2*w, w,w ), cv::Scalar(255,255,0), 1);
    	Palette pa_red(cv::Rect(offx, offy, w, w), cv::Scalar(255,0,0), -1);
    	Palette pa_green(cv::Rect(offx, offy+w*2, w, w), cv::Scalar(0,255,0), -1);
    	Palette pa_blue(cv::Rect(offx, offy+w*4, w, w), cv::Scalar(0,0,255), -1);
    	Palette pa_white(cv::Rect(offx, offy+w*6, w, w), cv::Scalar(255,255,255), -1);

    	palettes.clear();
    	palettes.push_back(pa_empty); // empty means to clear the whole result Mat, should be firstly pushed back
    	palettes.push_back(pa_red);
    	palettes.push_back(pa_green);
    	palettes.push_back(pa_blue);
        palettes.push_back(pa_white);

    }
public:
    Pen selectedPen;
    cv::Mat mat;
    cv::Rect validRegion;
    std::vector<Palette> palettes;
};


#endif