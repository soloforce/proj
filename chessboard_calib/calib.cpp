#include <iostream>
#include <fstream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#define CALIBRATION_IMAGE_NEEDED 8
#define CORNER_ROWS 9
#define CORNER_COLS 9

#define OPENCV3

void show_image_corners(std::vector<cv::Point2f>& corners)
{
    std::vector<cv::Point2f>::iterator it;
    for(it=corners.begin();it!=corners.end();it++){
        std::cout<<"("<<it->x<<","<<it->y<<")"<<"---";
    }
    std::cout<<std::endl;
}

void get_object_corners(std::vector<cv::Point3f> &corners, cv::Size s)
{
    for(int r=0; r<s.height; r++){
        for(int c=0; c<s.width; c++){
            corners.push_back(cv::Point3f(r,c,0));
        }
    }
}

void show_mat(cv::Mat mat)
{
    std::cout<<"rows:"<<mat.rows<<" cols:"<<mat.cols<<std::endl;
    for(int r=0;r<mat.rows;r++){
        for(int c=0;c<mat.cols;c++){
            std::cout<<mat.at<double>(r,c)<<" ";
        }
        std::cout<<std::endl;
    }
}



int main(int, char**)
{
    cv::VideoCapture cap(0); // open the default camera

    if(!cap.isOpened())  // check if we succeeded
    {
        std::cout<<"failed to open camera, try to load local images..."<<std::endl;
        //return -1;
    }
    std::cout<<"Press 'c' to start camera calibration!\n\tPress 'q' to quit!" <<std::endl;

#ifdef OPENCV3
    cap.set(cv::CAP_PROP_FRAME_WIDTH,640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,480);
#else
    cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
#endif

    cv::namedWindow("camera",1);


    int validImageCount=0;
    bool isCalibrated=false;

    cv::Size imageSize;
    std::vector< std::vector<cv::Point2f> > imagePoints;
    std::vector< std::vector<cv::Point3f> > objectPoints;


    cv::Mat map1,map2;
    cv::Mat calibratedImage;

    std::vector<cv::Point2f> imageCorners;
    cv::Size boardSize(CORNER_ROWS,CORNER_COLS);

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat>rvecs,tvecs;

    double apertureWidth, apertureHeight;
    double fovx,fovy;
    double focalLength,aspectRatio;
    cv::Point2d principalPoint;

    for(;;)
    {

        cv::Mat frame,gray;
        std::string framename;

        if(cap.isOpened()){
            cap >> frame; // get a new frame from camera
            if(frame.empty()) continue;
            cv::imshow("camera", frame);
        }



        int key_pressed = cv::waitKey(30);
        if( key_pressed == 'q' ) break;
        else if ( key_pressed == 'c' ){

            // if the camera is unvalible, use the local images
            if(!cap.isOpened()){
                char fname[16];
                sprintf(fname,"snap%02d.png", validImageCount);
                frame=cv::imread(fname);
            }

            imageSize=frame.size();
            if(validImageCount<CALIBRATION_IMAGE_NEEDED)
            {

                bool found=cv::findChessboardCorners(frame,boardSize,imageCorners);

                if(!found){
                    std::cout<<"Chessboard not found!"<<std::endl;
                }else{
                    char fname[16];
                    sprintf(fname,"snap%02d.png", validImageCount);
                    imwrite(fname,frame);

                    #ifdef OPENCV3
                    cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
                    #else
                    cv::cvtColor(frame,gray,CV_BGR2GRAY);
                    #endif



                    // Get subpixel accuracy on the corners
                    cv::cornerSubPix(gray, imageCorners,
                                        cv::Size(5,5),
                                        cv::Size(-1,-1),
                                        cv::TermCriteria(cv::TermCriteria::MAX_ITER +
                                        cv::TermCriteria::EPS,
                                        30,      // max number of iterations
                                        0.1));  // min accuracy


                    std::cout<<imageCorners<<"\n**********\n";


                    cv::drawChessboardCorners(frame,boardSize,imageCorners,found);
                    std::vector<cv::Point3f> objectCorners;
                    get_object_corners(objectCorners,boardSize);

                    imagePoints.push_back(imageCorners);
                    objectPoints.push_back(objectCorners);

                    validImageCount++;
                    std::cout<<"Still need to capture "<<CALIBRATION_IMAGE_NEEDED-validImageCount<<" chessboard images!"<<std::endl;
                    cv::imshow("camera", frame);
                    cv::waitKey(1000);
                }

            }

        }

        if(validImageCount>=CALIBRATION_IMAGE_NEEDED){
            if(!isCalibrated){
                // start calibration
                std::cout<<"Now, it's time to calibrate the camera!"<<std::endl;

                cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
                distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
#ifdef OPENCV3
                calibrateCamera(objectPoints,imagePoints,imageSize,
                                cameraMatrix,distCoeffs,rvecs,tvecs,cv::CALIB_FIX_K4|cv::CALIB_FIX_K5);
#else
                calibrateCamera(objectPoints,imagePoints,imageSize,
                                cameraMatrix,distCoeffs,rvecs,tvecs,CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
#endif

                isCalibrated=true;
                std::cout<<"The camera intrisic parameters:"<<std::endl;
                std::cout<<cameraMatrix<<std::endl;

                // Save the camera intrinsic parameters to file.
                std::cout<<"Saving camera matrix & distCoeffs ..."<<std::endl;
                cv::FileStorage fs("camera.xml", cv::FileStorage::WRITE );
                fs << "Camera_Matrix" << cameraMatrix;
                fs << "Distortion_Coefficients" << distCoeffs;


                calibrationMatrixValues(cameraMatrix,imageSize,apertureWidth,apertureHeight,fovx,fovy,focalLength,
                                    principalPoint,aspectRatio);

                std::cout   <<"=======\n"
                            <<"apertureWidth:"<<apertureWidth<<"\n"
                            <<"aperturetHeight:"<<apertureHeight<<"\n"
                            <<"fovx:"<<fovx<<"\n"
                            <<"fovy:"<<fovy<<"\n"
                            <<"focalLength:"<<focalLength<<"\n"
                            <<"aspectRatio:"<<aspectRatio<<"\n"
                            <<"principalPoint:"<<principalPoint.x<<","<<principalPoint.y<<"\n";

                cv::Mat rm;
                std::cout<<"The rotation matrixs:"<<std::endl;
                std::vector<cv::Mat>::iterator it;
                for(it=rvecs.begin();it!=rvecs.end();it++){
                    cv::Rodrigues(*it,rm);
                    std::cout<<rm<<std::endl;
                    std::cout<<"-----"<<std::endl;
                }

                std::cout<<"\nThe traslantion matrixs:"<<std::endl;
                for(it=tvecs.begin();it!=tvecs.end();it++){
                    std::cout<<*it<<std::endl;
                    std::cout<<"-----"<<std::endl;
                }

                // test solvephp()
                std::vector<cv::Point2f> imgPoints;
                std::vector<cv::Point3f> objPoints;
                imgPoints.push_back(cv::Point2f(1,1));
                imgPoints.push_back(cv::Point2f(1,2));
                imgPoints.push_back(cv::Point2f(2,2));
                imgPoints.push_back(cv::Point2f(2,1));

                objPoints.push_back(cv::Point3f(4,4,0));
                objPoints.push_back(cv::Point3f(4,8,0));
                objPoints.push_back(cv::Point3f(8,8,0));
                objPoints.push_back(cv::Point3f(8,4,0));

                cv::Mat rvec, rmat, tvec;
                solvePnP(objPoints,imgPoints,cameraMatrix,cv::Mat(),rvec,tvec);


                cv::Rodrigues(rvec,rmat);

                std::cout<<"\n\n"<<rmat<<"--\n"<<tvec<<"\n";


                std::cout<<"Calibration done, showing the undistorted image frame ..."<<std::endl;
            }


            if(cap.isOpened()){
                map1.create(frame.size(),CV_32FC1);
                map2.create(frame.size(),CV_32FC1);
                calibratedImage.create(frame.size(),CV_32FC1);

                cv::initUndistortRectifyMap(cameraMatrix,distCoeffs,cv::Mat(),
                                        cv::Mat(),frame.size(),CV_32FC1,map1,map2);



    #ifdef OPENCV3
                cv::remap(frame,calibratedImage,map1,map2,cv::INTER_LINEAR);
    #else
                cv::remap(frame,calibratedImage,map1,map2,CV_INTER_LINEAR);
    #endif



                cv::namedWindow("camera_undistored",1);
                cv::imshow("camera_undistored",calibratedImage);
            }else{
                std::cout<<"Camera parameters have been save to 'camera.xml', press 'q' to quit"<<std::endl;
            }
        }



    }



    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

