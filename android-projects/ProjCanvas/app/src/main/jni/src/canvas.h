#ifndef _CANVAS_H_
#define _CANVAS_H_

#include <opencv2/opencv.hpp>
#include <vector>

#include "bzpoint2f.h"
#include "widget.h"

/** Pen class.
*   Details:
*/
class Pen{
public:
    static const int LARGE_THICKNESS=8;
    static const int MEDIUM_THICKNESS=4;
    static const int SMALL_THICKNESS=1;
public:
    Pen():color(cv::Scalar(255,255,255)), thickness(1) { }
    Pen(cv::Scalar c, int w=1, int t=CV_AA):color(c),thickness(w), type(t){}
public:
    cv::Scalar color;
    int thickness;
    int type;
};

/** Canvas class.
*   Details:
*/
class Canvas{
public:
    Canvas(){
        selectedPen=Pen(cv::Scalar(255,255,255), 1);
        bgColor=cv::Scalar(0,0,0);
    }
    virtual ~Canvas(){
        for(int i=0; i<widgets.size(); i++){
            delete widgets[i];
        }
        widgets.clear();
    }
public:
    void setMat(cv::Mat& m){
        mat=m;
    }

    void setValidRegion(cv::Rect rect){
        validRegion=rect;
    }

    void drawBorder(Pen pen){
        cv::rectangle(mat, validRegion, pen.color, pen.thickness, pen.type);
    }

    void reset(){
        mat.setTo(bgColor);
        drawBorder(Pen(cv::Scalar(255,255,255), 1));
    }

    void clear(){
        mat.setTo(bgColor);
        drawBorder(Pen(cv::Scalar(0,255,0), 1));
        drawWidgets();
    }

    void drawCircle(cv::Point2f center, int radius, Pen pen){
        if(mat.empty()) return;
        cv::circle(mat, center, radius, pen.color, pen.thickness, pen.type);
    }

    void drawRectangle(cv::Rect rect, Pen pen)
    {
        if(mat.empty()) return;
        cv::rectangle(mat, rect, pen.color, pen.thickness, pen.type);
    }

    void drawLine(cv::Point2f prevPt, cv::Point2f pt, Pen pen){
        if(mat.empty()) return;
        cv::line(mat, prevPt, pt, pen.color, pen.thickness, pen.type);
    }

    void drawWidgets()
    {
        if(mat.empty()) return;

        for(int i=0; i<widgets.size(); i++){
            Widget* w=widgets[i];
            w->draw();
        }

    }

    void drawSelectedPen()
    {
        int i;
        ThicknessWidget* w=NULL;
        if(mat.empty()) return;
        for(int i=0; i<widgets.size(); i++){
            ThicknessWidget* temp=dynamic_cast<ThicknessWidget*>(widgets[i]);
            if(!temp)continue;
            if(temp->thickness == selectedPen.thickness ) { w=temp; }
            temp->draw();
        }

        if(w){
            // highlight the current selected pen properties
            if(selectedPen.color==cv::Scalar(255,255,255))
                drawRectangle(w->region, Pen(cv::Scalar(255,0,0), w->borderThickness));
            else
                drawRectangle(w->region, Pen(selectedPen.color, w->borderThickness));
            cv::Point2f center(w->region.tl().x+w->region.width/2, w->region.tl().y+w->region.height/2);
            drawCircle(center, w->radius, Pen(selectedPen.color, -1));
        }

    }

    bool selectPen(cv::Point2d pt)
    {
        int i;
        bool selected=false;
        PaletteWidget* pw;
        ThicknessWidget* tw;

    	if(widgets.size()) for(i=0; i<widgets.size(); i++){
    		if(!widgets[i]->region.contains(pt)) continue;
            else break;
    	}
        if( i>=widgets.size() ) return false;

        switch(widgets[i]->type){
        case Widget::TYPE_CLEAR:
            clear();
            break;
        case Widget::TYPE_PALETTE:
            pw=dynamic_cast<PaletteWidget*>(widgets[i]);
            if(pw) selectedPen.color= pw->innerColor;
            break;
        case Widget::TYPE_THICKNESS:
            tw=dynamic_cast<ThicknessWidget*>(widgets[i]);
            if(tw) selectedPen.thickness=tw->thickness;
            break;
        }

        drawSelectedPen();

        return true;
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
        const cv::Scalar blue(0,0,255);
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

    void initWidgets()
    {
    	const int w=50; // Widget width
    	const int m=20; // margin

    	// free the old widgets's memory
        for(int i=0; i<widgets.size(); i++){
            delete widgets[i];
        }
    	widgets.clear();

        // palettes widgets layouting
    	int offx=validRegion.br().x-(w+m);
    	int offy=validRegion.tl().y+m;
    	PaletteWidget *redPalette = new PaletteWidget (mat, cv::Rect(offx, offy, w, w), cv::Scalar(255,0,0));
    	PaletteWidget *greenPalette = new PaletteWidget(mat, cv::Rect(offx, offy+w+m, w, w), cv::Scalar(0,255,0));
    	PaletteWidget *bluePalette = new PaletteWidget(mat, cv::Rect(offx, offy+(w+m)*2, w, w), cv::Scalar(0,0,255));
    	PaletteWidget *yellowPalette = new PaletteWidget(mat, cv::Rect(offx, offy+(w+m)*3, w, w), cv::Scalar(255,255,0));
    	PaletteWidget *whitePalette = new PaletteWidget(mat, cv::Rect(offx, offy+(w+m)*4, w, w), cv::Scalar(255,255,255));
    	PaletteWidget *erasePalette = new PaletteWidget(mat, cv::Rect(offx, offy+(w+m)*5, w, w), bgColor);

        widgets.push_back(redPalette);
        widgets.push_back(greenPalette);
        widgets.push_back(bluePalette);
        widgets.push_back(yellowPalette);
        widgets.push_back(whitePalette);
        widgets.push_back(erasePalette);

        // clear button widget
    	ClearWidget *clearButton = new ClearWidget(mat, cv::Rect(validRegion.tl().x+m, validRegion.br().y-w-m, w,w ));
    	widgets.push_back(clearButton);

        // thickness button widgets
        offx=validRegion.br().x-w-m;
        offy=validRegion.br().y-w-m;
        ThicknessWidget *largeThickness =new ThicknessWidget(mat, cv::Rect(offx, offy, w, w), Pen::LARGE_THICKNESS, w/4);
        ThicknessWidget *mediumThickness=new ThicknessWidget(mat, cv::Rect(offx-w-m, offy, w, w), Pen::MEDIUM_THICKNESS, w/6);
        ThicknessWidget *smallThickness=new ThicknessWidget(mat, cv::Rect(offx-(w+m)*2, offy, w,w), Pen::SMALL_THICKNESS, w/10);
        widgets.push_back(largeThickness);
        widgets.push_back(mediumThickness);
        widgets.push_back(smallThickness);
    }
public:
    Pen selectedPen;
    cv::Mat mat;
    cv::Rect validRegion;
    std::vector<Widget*> widgets;
    cv::Scalar bgColor;

};


#endif