#include "bzinterpolating.hpp"

void onMouse(int event, int x, int y, int, void* extraData)
{

    BezierInterpolating& bzInterp=*((BezierInterpolating*)extraData);
    Canvas& canvas=bzInterp.getCanvas();


    if( event==cv::EVENT_LBUTTONDOWN){
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
        }
    }
    return 0;
}
