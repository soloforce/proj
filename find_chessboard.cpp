#include "opencv2/opencv.hpp"
#include "opencv2/core/types_c.h"

#define USE_CHESSBOARD 0
#define USE_CIRCLES 1

using namespace cv;
using namespace std;

void drawCircleCenters(cv::Mat& img, cv::Size s, vector<Point2f>& centers, bool isFound)
{
    if(!isFound){
        std::cout<<"Circle center not found!"<<std::endl;
        return;
    }

    if( (!img.data) || (s.width * s.height != centers.size()) ){
        std::cout<<"Circle center not drawed."<<std::endl;
        return;
    }

    for(int r=0; r<s.height; r++){
        for(int c=0; c<s.width; c++){
            Point pt=centers[r*s.width+c];
            circle(img, pt, 3, Scalar(0,255,0));
        }
    }
}


int main(int, char**)
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat edges;
    namedWindow("frame",1);

#if defined(USE_CHESSBOARD) && (USE_CHESSBOARD==1)
    Size patternsize(9,4);
    vector<Point2f> corners;
#elif defined(USE_CIRCLES) && (USE_CIRCLES==1)
    Size patternsize(9,8);
    vector<Point2f> centers;
#endif
    Mat gray;

    bool patternfound=false;

    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        if(!frame.data){
            std::cout<<"Invalid frame, skipped..."<<std::endl;
            continue;
        }

        cvtColor(frame, gray, COLOR_BGR2GRAY);

        #if defined(USE_CHESSBOARD) && (USE_CHESSBOARD==1)
            //CALIB_CB_FAST_CHECK saves a lot of time on images
            //that do not contain any chessboard corners
            patternfound = findChessboardCorners(gray, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE); //+ CALIB_CB_FAST_CHECK);
            if(patternfound)
                cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

            drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);

        #elif defined(USE_CIRCLES) && (USE_CIRCLES==1)
            patternfound = findCirclesGrid(gray, patternsize, centers, CALIB_CB_SYMMETRIC_GRID+CALIB_CB_CLUSTERING);
            //drawCircleCenters(frame, patternsize, centers, patternfound);
            drawChessboardCorners(frame, patternsize, Mat(centers), patternfound);
        #endif


        imshow("frame", frame);
        if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
