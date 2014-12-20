#ifndef _WIDGET_H_
#include "opencv2/opencv.hpp"

class Widget{
    friend class Canvas;
public:
    typedef enum{
        TYPE_PALETTE,
        TYPE_THICKNESS,
        TYPE_CLEAR,
        TYPE_INTERPOLATING
    }type_t;

public:
    Widget(cv::Mat& m, cv::Rect rect):mat(m), region(rect){
        borderColor=cv::Scalar(255,255,255);
        borderThickness=1;
        innerColor=cv::Scalar(255,255,255);
    }
    virtual void draw(){}
    virtual ~Widget(){}
    void setType(type_t t){ type=t; }
    void setRegion(cv::Rect rect) { region=rect; }
    void setInnerColor(cv::Scalar color){ innerColor=color; }
    void setBorderColor(cv::Scalar color){ borderColor=color; }
    void setBorderThickness( int th ) { borderThickness=th; }
protected:
    type_t type;
    cv::Rect region;
    cv::Scalar borderColor;
    cv::Scalar innerColor;
    cv::Mat& mat;
    int borderThickness;
};

class PaletteWidget : public Widget{
    friend class Canvas;
public:
    PaletteWidget(cv::Mat& m, cv::Rect rect, cv::Scalar color): Widget(m,rect){
        innerColor=color;
        type=TYPE_PALETTE;
    }

        
    virtual void draw(){

        cv::Rect innerRegion;
        if(mat.empty())return;

        innerRegion=region;
        innerRegion.x += borderThickness;
        innerRegion.y += borderThickness;
        innerRegion.width -= borderThickness*2;
        innerRegion.height -= borderThickness*2;

        cv::rectangle(mat, region, borderColor, borderThickness, CV_AA);
        cv::rectangle(mat, innerRegion, innerColor, -1, CV_AA);
    }
};

class ClearWidget : public Widget{
    friend class Canvas;
public:
    ClearWidget(cv::Mat& m, cv::Rect rect):Widget(m, rect){
        type=TYPE_CLEAR;
        innerColor=cv::Scalar(255,255,0);
        borderColor=cv::Scalar(255,255,0);
    }

    virtual void draw(){
        if(mat.empty()) return;
        cv::rectangle(mat, region, borderColor, borderThickness, CV_AA);
        cv::Point2f tr(region.br().x, region.tl().y);
        cv::Point2f bl(region.tl().x, region.br().y);
        cv::line(mat, region.tl(), region.br(), innerColor, borderThickness, CV_AA);
        cv::line(mat, tr, bl, innerColor, borderThickness, CV_AA);

    }
};

class ThicknessWidget: public Widget{
    friend class Canvas;
public:
    ThicknessWidget(cv::Mat& m, cv::Rect rect, int t, int r):Widget(m, rect), thickness(t),radius(r){
        type=TYPE_THICKNESS;
        innerColor=cv::Scalar(255,255,255);
    }
    virtual void draw(){
        if(mat.empty()) return;
        cv::rectangle(mat, region, borderColor, borderThickness, CV_AA);
        cv::Point2f center=cv::Point2f(region.tl().x+region.width/2, region.tl().y+region.height/2);
        cv::circle(mat, center, radius, innerColor, -1, CV_AA);
    }
protected:
    int thickness;
    int radius;
};

class InterpolatingWidget: public Widget{
    friend class Canvas;
public:
    InterpolatingWidget(cv::Mat& m, cv::Rect rect, bool flag):Widget(m, rect),  interpEnabled(flag){
        type=TYPE_INTERPOLATING;
    }
    virtual void draw(){
        if(mat.empty()) return;
        cv::Point2f center=cv::Point2f(region.tl().x+region.width/2, region.tl().y+region.height/2);
        int radius=region.width/3.0;
        cv::rectangle(mat, region, cv::Scalar(255,255,255), borderThickness, CV_AA);
        if(interpEnabled)
            cv::circle(mat, center, radius, borderColor, borderThickness, CV_AA);
        else
            cv::line(mat, region.tl(), region.br(), cv::Scalar(255,255,255), borderThickness, CV_AA);
    }
protected:
    bool interpEnabled;
};

#endif