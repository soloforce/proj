#ifndef _CANVAS_H_
#define _CANVAS_H_

#include <opencv2/opencv.hpp>
#include <vector>


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
        selectedPen=Pen(cv::Scalar(255,255,255), 2);
    }

public:
    void setCanvas(cv::Mat& m){
        canvas=m;
    }

    void setValidRegion(cv::Rect rect){
        validRegion=rect;
    }

    void drawRegion(Pen pen){
        cv::rectangle(canvas, validRegion, pen.color, pen.thickness, pen.type);
    }

    void clear(){
        canvas.setTo(cv::Scalar(0,0,0));
    }

    void reset(){
        selectedPen=Pen(cv::Scalar(255,255,255), 2);
        canvas.setTo(cv::Scalar(0,0,0));
        drawRegion(Pen(cv::Scalar(255,255,255), 1));
        drawCircle(validRegion.tl(), 3, Pen(cv::Scalar(255,0,0), -1));
    }

    void drawCircle(cv::Point center, int radius, Pen pen){
        if(canvas.empty()) return;
        cv::circle(canvas, center, radius, pen.color, pen.thickness, pen.type);
    }

    void drawRectangle(cv::Rect rect, Pen pen)
    {
        if(canvas.empty()) return;
        cv::rectangle(canvas, rect, pen.color, pen.thickness, pen.type);
    }

    void drawLine(cv::Point prevPt, cv::Point pt, Pen pen){
        if(canvas.empty()) return;
        cv::line(canvas, prevPt, pt, pen.color, pen.thickness, pen.type);
    }

    void drawPalettes()
    {
        if(canvas.empty()) return;
        for(int i=0; i<palettes.size(); i++)
            drawRectangle( palettes[i].rect, Pen(palettes[i].color,  palettes[i].thickness));
    }

    void drawSelectedColor()
    {
        if(canvas.empty()) return;
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
    		}else selectedPen=Pen(palettes[i].color, 2);
    		drawSelectedColor();
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
    cv::Mat canvas;
    cv::Rect validRegion;
    std::vector<Palette> palettes;
};


#endif