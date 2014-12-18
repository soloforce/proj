

//Access SimpleBlobDetector datas for video

#include "opencv2/imgproc/imgproc.hpp" //
#include "opencv2/highgui/highgui.hpp"

    #include <iostream>
    #include <math.h>
    #include <vector>
    #include <fstream>
    #include <string>
    #include <sstream>
    #include <algorithm>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/features2d/features2d.hpp"


using namespace cv;
using namespace std;


int main(int argc, char *argv[])
{


    const char* fileName ="v.mp4";
    VideoCapture cap(fileName); //
    if(!cap.isOpened()) //
    {
        //cout << "Couldn't open Video  " << fileName << "\n";
        return -1;
    }
    for(;;)  // videonun frameleri icin sonsuz dongu
    {
        Mat frame,labelImg;
        cap >> frame;
        if(frame.empty()) break;
        //imshow("main",frame);

        Mat frame_gray;
        cvtColor(frame,frame_gray,COLOR_RGB2GRAY);


        //////////////////////////////////////////////////////////////////////////
        // convert binary_image
        Mat binaryx;
        threshold(frame_gray,binaryx,120,255,THRESH_BINARY);


        Mat src, gray, thresh, binary;
        Mat out;
        vector<KeyPoint> keyPoints;

        SimpleBlobDetector::Params params;
        params.minThreshold = 125;
        params.maxThreshold = 255;
        params.thresholdStep = 100;

        params.minArea = 10;
        params.minConvexity = 0.3;
        params.minInertiaRatio = 0.01;

        params.maxArea = 1000;
        params.maxConvexity = 10;

        params.filterByColor = false;
        params.filterByCircularity = false;



        src = binaryx.clone();

        SimpleBlobDetector blobDetector( params );
        blobDetector.create("SimpleBlob");



        blobDetector.detect( src, keyPoints );
        drawKeypoints( src, keyPoints, out, Scalar(255,0,0), DrawMatchesFlags::DEFAULT);


        cv::Mat blobImg;
        cv::drawKeypoints(frame, keyPoints, blobImg);
        cv::imshow("Blobs", blobImg);

        for(int i=0; i<keyPoints.size(); i++){
            //circle(out, keyPoints[i].pt, 20, cvScalar(255,0,0), 10);
            //cout<<keyPoints[i].response<<endl;
            //cout<<keyPoints[i].angle<<endl;
            //cout<<keyPoints[i].size()<<endl;
            cout<<keyPoints[i].pt.x<<endl;
            cout<<keyPoints[i].pt.y<<endl;

        }
        imshow( "out", out );

        if ((waitKey(40)&0xff)==27) break;  // esc 'ye basilinca break
    }

}

