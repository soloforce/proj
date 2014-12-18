#include "bzinterpolating.hpp"

double lastTick;
static int tickUpdateTimeLimit=200; // in ms

void onMouse(int event, int x, int y, int flag, void* extraData)
{
    static bool tickNeedToUpdate=true;

    BezierInterpolating& bzInterp=*((BezierInterpolating*)extraData);
    Canvas& canvas=bzInterp.getCanvas();

    if( event == cv::EVENT_LBUTTONDOWN ){
        // triggered when left-button-pressed drag
        BZPoint2f cur(x,y);
        if(bzInterp.empty()){
            canvas.drawCircle(cur, 1 , cv::Scalar(0,0,0), -1);
        }else{
            BZPoint2f pre= bzInterp[bzInterp.size()-1];
            canvas.drawLine(pre, cur,  cv::Scalar(0,0,0), 1);
        }
        bzInterp.push_back(cur);
    }else if( (event== cv::EVENT_MOUSEMOVE) && (flag & cv::EVENT_FLAG_RBUTTON) ){

        if(tickNeedToUpdate){
            lastTick=cv::getTickCount();
            tickNeedToUpdate=false;
        }
        double t=(cv::getTickCount()-lastTick)/(cv::getTickFrequency())*1000;
        if(t<tickUpdateTimeLimit) return;
        tickNeedToUpdate=true;

        // triggered when left-button-pressed drag
        BZPoint2f cur(x,y);
        if(bzInterp.empty()){
            canvas.drawCircle(cur, 1 , cv::Scalar(0,0,0), -1);
        }else{
            BZPoint2f pre= bzInterp[bzInterp.size()-1];
            canvas.drawLine(pre, cur,  cv::Scalar(0,0,0), 1);
        }
        bzInterp.push_back(cur);
    }else if( event==cv::EVENT_RBUTTONDOWN ){
        bzInterp.clear();
    }
}

int main()
{
    BezierInterpolating bzInterp;
    Canvas& canvas=bzInterp.createCanvas(Canvas::HEIGHT, Canvas::WIDTH );

    cv::namedWindow("Canvas", 0);
    cv::setMouseCallback("Canvas", onMouse, &bzInterp);
    bool isRunning=true;
    bool isControlPointsVisible=false;
    cv::Mat oriMat;

    while(isRunning){
        cv::imshow("Canvas", canvas.getMat());

        int c=cv::waitKey(30);

        switch( c&255 ){
        case 27: //ESC key pressed
            std::cout<<"Exiting ..."<<std::endl;
            isRunning=false;
            break;
        case 'b':
            bzInterp.calcAllControlPoints();
            bzInterp.calcAllBezierPoints();
            canvas.drawInterpolatedPoints(bzInterp.getBezierPoints(), cv::Scalar(0,0,255), 1, isControlPointsVisible);
            break;
        case 'c':
            bzInterp.clear();
            canvas.clear();
            break;
        case 'v':
            isControlPointsVisible=!isControlPointsVisible;
            bzInterp.setControlPointsVisibility(isControlPointsVisible);
            //oriMat=canvas.getMat().clone();
            canvas.clear();
            //canvas.setMat(oriMat);
            canvas.drawInterpolatedPoints(bzInterp.getBezierPoints(), cv::Scalar(0,0,255), 1, isControlPointsVisible);
            break;
        case 'o':
            canvas.drawOriginalPoints(bzInterp.getBezierPoints(), cv::Scalar(100,100,100), 1);
            break;
        case '=':
            tickUpdateTimeLimit*=2;
            break;
        case '-':
            tickUpdateTimeLimit/=2;
            break;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            BZPoint2f::globalSmoothFactor= ((c&255)-'0')/10.0;
            bzInterp.calcAllControlPoints();
            bzInterp.calcAllBezierPoints();
            break;
        case '0':
            BZPoint2f::globalSmoothFactor= 1.0;
            bzInterp.calcAllControlPoints();
            bzInterp.calcAllBezierPoints();
            break;
        }
    }
    return 0;
}
