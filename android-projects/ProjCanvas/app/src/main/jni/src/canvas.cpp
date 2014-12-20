#include "canvas.h"



Canvas::Canvas()
{
    selectedPen=Pen(cv::Scalar(255,255,255), 1);
    bgColor=cv::Scalar(0,0,0);
    interpEnabled=true;
}

Canvas::~Canvas(){
    for(int i=0; i<widgets.size(); i++)
    {
        delete widgets[i];
    }
    widgets.clear();
}

void Canvas::setMat(cv::Mat& m)
{
    mat=m;
}

void Canvas::setValidRegion(cv::Rect rect)
{
    validRegion=rect;
}

void Canvas::drawBorder(Pen pen)
{
    cv::rectangle(mat, validRegion, pen.color, pen.thickness, pen.type);
}

void Canvas::reset()
{
    mat.setTo(bgColor);
    drawBorder(Pen(cv::Scalar(255,255,255), 1));
}

void Canvas::clear()
{
    mat.setTo(bgColor);
    drawBorder(Pen(cv::Scalar(0,255,0), 1));
    drawWidgets();
    drawSelectedPen();
}

void Canvas::drawCircle(cv::Point2f center, int radius, Pen pen)
{
    if(mat.empty()) return;
    cv::circle(mat, center, radius, pen.color, pen.thickness, pen.type);
}

void Canvas::drawRectangle(cv::Rect rect, Pen pen)
{
    if(mat.empty()) return;
    cv::rectangle(mat, rect, pen.color, pen.thickness, pen.type);
}

void Canvas::drawLine(cv::Point2f prevPt, cv::Point2f pt, Pen pen){
    if(mat.empty()) return;
    cv::line(mat, prevPt, pt, pen.color, pen.thickness, pen.type);
}

void Canvas::drawWidgets()
{
    if(mat.empty()) return;

    for(int i=0; i<widgets.size(); i++){
        Widget* w=widgets[i];
        w->draw();
    }

}

void Canvas::drawSelectedPen()
{
    int i;
    ThicknessWidget* thick=NULL;
    InterpolatingWidget* interp=NULL;
    if(mat.empty()) return;
    for(int i=0; i<widgets.size(); i++){
        widgets[i]->draw();
        ThicknessWidget* tw=dynamic_cast<ThicknessWidget*>(widgets[i]);
        InterpolatingWidget* iw=dynamic_cast<InterpolatingWidget*>(widgets[i]);
        if(tw && (tw->thickness == selectedPen.thickness) ) { thick=tw; }
        if(iw && (iw->interpEnabled == interpEnabled) ) { interp=iw; }
    }

    // highlight the current selectedPen's properties
    if(thick){
        // highlight the border
        drawRectangle(thick->region, Pen(cv::Scalar(0,255,0), thick->borderThickness));
        // highlight the circle
        cv::Point2f center(thick->region.tl().x+thick->region.width/2, thick->region.tl().y+thick->region.height/2);
        drawCircle(center, thick->radius, Pen(selectedPen.color, -1));
    }

    // interpolating enabled or not, highlight the button related
    if(interp){
        drawRectangle(interp->region, Pen(cv::Scalar(0,255,0), interp->borderThickness));
    }

}

bool Canvas::selectPen(cv::Point2d pt)
{
    int i;
    bool selected=false;
    PaletteWidget* pw;
    ThicknessWidget* tw;
    InterpolatingWidget* iw;

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
    case Widget::TYPE_INTERPOLATING:
        iw=dynamic_cast<InterpolatingWidget*>(widgets[i]);
        if(iw){
            interpEnabled=iw->interpEnabled;
        }
    }

    drawSelectedPen();

    return true;
}

void Canvas::drawOriginalPoints(std::vector<BZPoint2f>& pts, Pen& pen)
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

void Canvas::drawInterpolatedPoints(std::vector<BZPoint2f>& pts, Pen& pen)
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

void Canvas::initWidgets()
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

    // interpolating widget
    InterpolatingWidget * enableButton=new InterpolatingWidget(mat,cv::Rect(offx-(w+m)*4, offy,w,w), true);
    InterpolatingWidget * disableButton=new InterpolatingWidget(mat,cv::Rect(offx-(w+m)*5, offy,w,w), false);
    widgets.push_back(enableButton);
    widgets.push_back(disableButton);

}

