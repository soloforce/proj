#include "bzinterpolating.hpp"

void onMouse(int event, int x, int y, int, void* extraData)
{

    BezierInterpolating& bzInterp=*((BezierInterpolating*)extraData);
    Canvas& canvas=bzInterp.getCanvas();


    if( event==EVENT_LBUTTONDOWN){
        BZPoint2f cur(x,y);
        if(bzInterp.empty()){
            canvas.drawCircle(cur, 1 , Scalar(0,0,0), -1);
        }else{
            BZPoint2f pre= bzInterp[bzInterp.size()-1];
            canvas.drawLine((Point2f)pre, (Point2f)cur,  Scalar(0,0,0), 1);
        }
        bzInterp.push_back(cur);
    }else if( event==EVENT_RBUTTONDOWN ){
        bzInterp.clear();
    }
}

int main()
{
    BezierInterpolating bzInterp;
    Canvas& canvas=bzInterp.createCanvas(Canvas::HEIGHT, Canvas::WIDTH );

    namedWindow("Canvas", 0);
    setMouseCallback("Canvas", onMouse, &bzInterp);
    bool isRunning=true;
    bool isControlPointsVisible=false;
    cv::Mat oriMat;

    while(isRunning){
        imshow("Canvas", canvas.getMat());
        int c=waitKey(30);

        switch( c&255 ){
        case 27: //ESC key pressed
            cout<<"Exiting ..."<<endl;
            isRunning=false;
            break;
        case 'b':
            bzInterp.calcAllControlPoints();
            bzInterp.calcAllBezierPoints();
            canvas.drawInterpolatedPoints(bzInterp.getBezierPoints(), Scalar(0,0,255), 1, isControlPointsVisible);
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
            canvas.drawInterpolatedPoints(bzInterp.getBezierPoints(), Scalar(0,0,255), 1, isControlPointsVisible);
            break;
        case 'o':
            canvas.drawOriginalPoints(bzInterp.getBezierPoints(), Scalar(100,100,100), 1);
            break;
        }
    }
    return 0;
}
