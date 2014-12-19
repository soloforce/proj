#include "processor.h"
#include "utils.h"


void Processor::reset()
{
    calib.reset();
    state.reset();
    bzInterp.clear();
    canvas.clear();
    inited=false;
}

void Processor::init(cv::Mat& result, cv::Rect rectCamera, cv::Rect rectCanvas)
{
    setValidRegion(rectCamera);

    canvas.setMat(result);
    canvas.setValidRegion(rectCanvas);
    canvas.initPalettes();

    calib.targetPoints.push_back(rectCanvas.tl());
    calib.targetPoints.push_back(cv::Point2d(rectCanvas.br().x, rectCanvas.y));
    calib.targetPoints.push_back(rectCanvas.br());
    calib.targetPoints.push_back(cv::Point2d(rectCanvas.x,rectCanvas.br().y));

    inited=true;
}


void Processor::getTouchedPoints(cv::Mat & frame, std::vector<cv::Point2d>& touchedPoints)
{
    int n;
    std::vector< std::vector<cv::Point> > contours;
    cv::Mat gray(frame.size(), CV_8UC1);
    cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
    cv::threshold(gray, gray, 50, 255, cv::THRESH_BINARY);
    cv::findContours(gray,contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    n=contours.size();
    if(n>1) std::sort(contours.begin(), contours.end(), vector_size_comp);
    if(n>Processor::MAX_TOUCHED_POINTS) n=Processor::MAX_TOUCHED_POINTS;
    for(int i=0; i<n; i++){
        touchedPoints.push_back(getCentroid(contours[i]));
    }

}

void Processor::process(cv::Mat& frame)
{

    std::vector<cv::Point2d> touchedPoints;
    if(frame.empty()) return;
    getTouchedPoints(frame, touchedPoints);
    state.change(touchedPoints.size());

    if(touchedPoints.size()){

        state.ptTouched=touchedPoints[0]; //! get the first touched point

        if(!calib.isCalibrated){
            canvas.clear();
            canvas.drawCircle(state.ptTouched, 3, Pen(cv::Scalar(255,255,255),-1));
        }

    }else if((!calib.isCalibrated) && validRegion.contains(state.ptTouched)){

        if(calib.cameraPoints.size()<Calibrator::CALIB_POINTS_COUNT){
            calib.cameraPoints.push_back(state.ptTouched);
        }

        if(calib.cameraPoints.size()>=Calibrator::CALIB_POINTS_COUNT){
            calib.calibrate();
            calib.isCalibrated=true;
            canvas.clear();
            canvas.drawRegion(Pen(cv::Scalar(255,255,255), 1));

        }
        state.ptTouched=cv::Point2d(-1,-1);
    }


    if(calib.isCalibrated && validRegion.contains(state.ptTouched)){
        cv::Mat m(3,1,CV_64FC1);
        point2mat(state.ptTouched, m);
        state.ptMapped=mat2point(calib.homo*m);

        if( canvas.validRegion.contains(state.ptMapped) ){
            drawAction();

            if(state.penState==State::PEN_UP){
                state.change(State::PEN_UP, State::PEN_NA);
                state.ptTouched=cv::Point2d(-1,-1);
            }

        }
    }

    if(!calib.isCalibrated){
        canvas.drawRegion(Pen(cv::Scalar(255,255,255), 1));
        canvas.drawCircle(canvas.validRegion.tl(), 3, Pen(cv::Scalar(255,0,0), -1));
    }else{
        canvas.drawRegion(Pen(cv::Scalar(0,255,0), 1));
        canvas.drawPalettes();
        canvas.drawSelectedColor();
    }
}

void Processor::drawAction()
{
    const int radius=1;
    cv::Point2f pt;
    cv::Scalar bgColor(0,0,0);
    Pen erasePen=canvas.selectedPen;
    erasePen.color=bgColor;

	switch(state.penState){
	case State::PEN_NA:
	    break;
	case State::PEN_UP:

        bzInterp.calcAllControlPoints();
        bzInterp.calcAllBezierPoints();

        // erase the original uninterpolated curve
        canvas.drawOriginalPoints(bzInterp.getBezierPoints(), erasePen);

        // draw the interpolated curve
        canvas.drawInterpolatedPoints(bzInterp.getBezierPoints(), canvas.selectedPen);

        // clear the last curve
        bzInterp.clear();
	    break;
	case State::PEN_DOWN:
	     pt=state.ptMapped;
	    bzInterp.push_back( pt);

		state.ptPrevMapped=state.ptMapped;
		canvas.selectPen(state.ptMapped);
		canvas.drawCircle(state.ptMapped, radius, canvas.selectedPen);
		break;
	case State::PEN_MOVE:
        pt=state.ptMapped;
	    bzInterp.push_back( pt);

	    canvas.drawLine(state.ptPrevMapped, state.ptMapped, canvas.selectedPen);
		state.ptPrevMapped=state.ptMapped;
		break;

	}
}