#include <iostream>
#include <cstdio>
#include <vector>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

#define CORNER_ROWS 9
#define CORNER_COLS 4
#define SNAP_MIN 1
#define SNAP_MAX 12


vector< vector<Point2f> > cam_imgPoints;
vector< vector<Point3f> > cam_objPoints;
vector< vector<Point2f> > proj_capturedPoints;
vector< vector<Point2f> > proj_patternPoints;
vector< vector<Point3f> > proj_objPoints;

// intrinsic parameters
// intrinsic matrix is the form like this:
//      fx  r cx
//      0  fy cy
//      0   0  1
Mat cameraMatrix,projectorMatrix;

// extended intrinsic parameters
// intrinsic matrix is extended into the form like this:
//      fx  r cx  0
//      0  fy cy  0
//      0   0  1  0
//      0   0  0  1
Mat cameraMatrixExt,projectorMatrixExt;

// extrinsic parameters(Rotation & Translation matrix)
// RT matrix is the form like this:
//      r11 r12 r13 t1
//      r21 r22 r23 t2
//      r31 r32 r33 t3
vector<Mat> cameraRT, projectorRT;

// extended extrinsic parameters(Rotation & Translation matrix)
// RT matrix is extended into the form like this:
//      r11 r12 r13 t1
//      r21 r22 r23 t2
//      r31 r32 r33 t3
//       0   0   0   1
vector<Mat> cameraRTExt, projectorRTExt;

bool gen_proj_pattern_points(std::vector<Point2f>& v, cv::Rect roi, cv::Size density)
{

    int delta_x=roi.width / (density.width-1);
    int delta_y=roi.height / (density.height-1);

    for(int r=0; r<=roi.height;r+=delta_y){
        for(int c=0;c<=roi.width;c+=delta_x){
            v.push_back(cv::Point2f(roi.x+c,roi.y+r));
        }
    }
    return true;
}

Point2f GetHeart(vector<Point>& v)
{
    Point2f pt(0,0);
    int n=0;

    vector<cv::Point>::iterator it;
    for(it=v.begin();it!=v.end();it++){
        pt.x+=(*it).x;
        pt.y+=(*it).y;
        n++;
    }

    if(n){
        pt.x=pt.x/n;
        pt.y=pt.y/n;
    }

    return pt;
}

int main()
{
    char fname1[16],fname2[16];
    Mat image,gray,bin;

    Size boardSize(CORNER_ROWS,CORNER_COLS); // chessboard size, number of corners
    Size imageSize; // image(captured by camera) size, pixels
    vector<Point2f> imgCorners; // corners coords vector

    Mat distCoeffs,proj_distCoeffs;
    vector<Mat>rvecs,tvecs,proj_rvecs,proj_tvecs;



    // -------------------------------------------------------------------------
    for(int i=SNAP_MIN;i<=SNAP_MAX;i++){
        sprintf(fname1,"snap%02d.png",i);
        image=imread(fname1);
        if(image.data==NULL){
            cout<<"failed to open image file!"<<endl;
            continue;
        }
        bool found=findChessboardCorners(image,boardSize,imgCorners);
        if(!found){
            cout<<"no chessboard found!"<<endl;
            continue;
        }

        cvtColor(image,gray,CV_BGR2GRAY);
        cornerSubPix(gray, imgCorners,
                    cv::Size(5,5),
                    cv::Size(-1,-1),
                    cv::TermCriteria(cv::TermCriteria::MAX_ITER +
                                cv::TermCriteria::EPS,
                                30,      // max number of iterations
                                0.1));   // min accuracy


        // generate chessboard-centered corner coords
        vector<Point3f> objCorners;
        for(int r=0; r<boardSize.height; r++){
            for(int c=0; c<boardSize.width; c++){
                objCorners.push_back(cv::Point3f(r,c,0));
            }
        }

        // push the correspondence into stack
        cam_imgPoints.push_back(imgCorners);
        cam_objPoints.push_back(objCorners);

    }

    // -------------------------------------------------------------------------
    // camera calibration
    imageSize=image.size();
    calibrateCamera(cam_objPoints,cam_imgPoints,imageSize,
                    cameraMatrix,distCoeffs,rvecs,tvecs,0);
    std::cout<<" *** The camera intrisic parameters:"<<std::endl;
    std::cout<<cameraMatrix<<std::endl;

    cameraMatrixExt=Mat::eye(4,4,CV_64F);
    Mat roi(cameraMatrixExt,Rect(0,0,3,3));
    cameraMatrix.copyTo(roi);

    std::cout<<" *** The camera extended intrisic parameters:"<<std::endl;
    std::cout<<cameraMatrixExt<<std::endl;

    // undistorting the image
    Mat map1,map2,undist;
    map1.create(image.size(),CV_32FC1);
    map2.create(image.size(),CV_32FC1);
    undist.create(image.size(),CV_32FC1);

    cv::initUndistortRectifyMap(cameraMatrix,distCoeffs,Mat(),
                        Mat(),image.size(),CV_32FC1,map1,map2);


    Mat kernel=getStructuringElement(MORPH_ELLIPSE, Size(3,3));

    for(int i=SNAP_MIN;i<=SNAP_MAX;i++){
        sprintf(fname1,"snap%02d.png",i);
        sprintf(fname2,"extract%02d.png",i);

        image=imread(fname1);
        remap(image,undist,map1,map2,CV_INTER_LINEAR);

        cvtColor(undist, gray, CV_BGR2GRAY);
        threshold(gray,bin,210,255,cv::THRESH_BINARY);
        dilate(bin,bin,kernel);

        // 寻找轮廓
        vector< std::vector<cv::Point> > contours;
        findContours(bin,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
        //drawContours(bin,contours,-1,Scalar(255),1);
        //cv::Mat heart_img(image.size(),CV_8UC3,cv::Scalar(0,0,0));

        vector<Point2f> pts;
        vector< std::vector<cv::Point> >::iterator it;
        for(it=contours.begin();it!=contours.end();it++){
            Point2f heart=GetHeart(*it);
            pts.push_back(heart);
            //circle(heart_img,heart,2,Scalar(0,0,255),CV_FILLED);
            //cout<<heart<<endl;
        }

        proj_capturedPoints.push_back(pts);

        //imwrite(fname2,heart_img);

    }

    // -------------------------------------------------------------------------
    // pack camera RT matrixes
    {
        Mat r,t;
        Mat rt;
        Mat roi;

        vector<Mat>::iterator it1, it2;
        it1=rvecs.begin();
        it2=tvecs.begin();
        for(int i=SNAP_MIN;i<=SNAP_MAX;i++){
            rt=Mat::eye(4,4,CV_64F);

            cv::Rodrigues(*it1,r);
            //cout<<r<<"\n\n";

            roi=cv::Mat(rt,cv::Rect(0,0,3,3));
            r.copyTo(roi);

            roi=cv::Mat(rt,cv::Rect(3,0,1,3));
            t=*it2;
            t.copyTo(roi);

            cameraRTExt.push_back(rt.clone());
            cameraRT.push_back(Mat(rt,cv::Rect(0,0,4,3)).clone());

            it1++;
            it2++;
        }
    }
    // -------------------------------------------------------------------------
    vector<Point2f> vPt;

    gen_proj_pattern_points(vPt,cv::Rect(100,100,600,300),cv::Size(7,4));
    proj_patternPoints.push_back(vPt);
    proj_patternPoints.push_back(vPt);
    proj_patternPoints.push_back(vPt);
    vPt.clear();

    gen_proj_pattern_points(vPt,cv::Rect(580,100,600,300),cv::Size(7,4));
    proj_patternPoints.push_back(vPt);
    proj_patternPoints.push_back(vPt);
    proj_patternPoints.push_back(vPt);
    vPt.clear();

    gen_proj_pattern_points(vPt,cv::Rect(100,400,600,300),cv::Size(7,4));
    proj_patternPoints.push_back(vPt);
    proj_patternPoints.push_back(vPt);
    proj_patternPoints.push_back(vPt);
    vPt.clear();

    gen_proj_pattern_points(vPt,cv::Rect(580,400,600,300),cv::Size(7,4));
    proj_patternPoints.push_back(vPt);
    proj_patternPoints.push_back(vPt);
    proj_patternPoints.push_back(vPt);
    vPt.clear();

    //cout<<proj_patternPoints<<endl;
    {
        for(int i=SNAP_MIN-1;i<SNAP_MAX;i++)
        {
            //cout<<*it<<endl;
            Mat N=Mat(cameraRT.at(i),cv::Rect(2,0,1,3));
            Mat P=Mat(cameraRT.at(i),cv::Rect(3,0,1,3));
            //cout<<N<<"\n"<<P<<"\n"<<N.dot(P)<<"\n\n";

            Mat RT_ext=cameraRTExt.at(i);
            Mat A_ext=cameraMatrixExt;

            vector<Point3f> v_proj;

            vector<Point2f> v=proj_capturedPoints.at(i);
            for(vector<Point2f>::iterator it=v.begin();it!=v.end();it++){
                Mat r=Mat::ones(4,1,CV_64F);
                r.at<double>(0,0)=it->x;
                r.at<double>(1,0)=it->y;

                Mat Rxyz=RT_ext.inv()*A_ext.inv()*r;
                //cout<<Rxyz<<endl;

                double s=N.dot(P)/N.dot(Mat(Rxyz,Rect(0,0,1,3)));
                Rxyz=s*Rxyz;
                //cout<<s<<endl<<endl;

                Point3f pt;
                pt.x=Rxyz.at<double>(0);
                pt.y=Rxyz.at<double>(1);
                pt.z=Rxyz.at<double>(2);

                v_proj.push_back(pt);



            }
            //cout<<v_proj.size()<<endl;
            //cout<<v_proj<<endl;

            proj_objPoints.push_back(v_proj);
            v_proj.clear();

        }
    }

    //-------------------------------------------------------------------------
    // projector calibration
    //-------------------------------------------------------------------------
    /*
    double m[3][3]={1280.0, 0,      640,
                    0,      800.0, 400,
                    0,      0,      1};
    projectorMatrix = cv::Mat(3, 3, CV_64FC1,m);
    proj_distCoeffs = cv::Mat::zeros(8, 1, CV_64FC1);

    calibrateCamera(proj_objPoints,proj_patternPoints,cv::Size(1280,800),
                    projectorMatrix,proj_distCoeffs,proj_rvecs,proj_tvecs,CV_CALIB_USE_INTRINSIC_GUESS);
    std::cout<<" *** The projector intrisic parameters:"<<std::endl;
    std::cout<<projectorMatrix<<std::endl;


    Mat r;
    cv::Rodrigues(proj_rvecs.at(0),r);
    cout<<r<<endl;
*/
    //-------------------------------------------------------------------------

    cout<<cam_imgPoints.at(1).at(0)<<endl;
    cout<<cam_objPoints.at(1).at(0)<<endl;
    cout<<cameraRT.at(1)<<endl;
}
