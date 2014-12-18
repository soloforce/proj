#include <iostream>
#include <cstdio>
#include <vector>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;


#define CORNER_ROWS 9
#define CORNER_COLS 4
#define SQUARE_SIZE 1.0
#define SNAP_MIN 1
#define SNAP_MAX 12



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
//Mat cameraMatrixExt,projectorMatrixExt;

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
//vector<Mat> cameraRTExt, projectorRTExt;

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


/**
 * Get a number of point's geometry heart
 *
 * @param v INPUT, vectort of points
 *
 * @return the heart with type Point2f
 */
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

    pt.x=pt.x/n;
    pt.y=pt.y/n;

    return pt;
}

/**
 * Set chessboard & squares sizes
 *
 * @param boardSize INPUT, rows & columns of squares' corners in a chessboard
 * @param suqareSize INPUT, squares' side length metric, like xxx mm(s)
 *                  or yyy cm(s), or normalized unit, like 1.0
 * @param objCorners OUPUT, coordinates of the corners
 *
 * @return none
 */
void setChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& objCorners)
{
    for( int i = 0; i < boardSize.height; i++ )
        for( int j = 0; j < boardSize.width; j++ )
            objCorners.push_back(Point3f(float(j*squareSize),float(i*squareSize), 0));
    
}

/**
 * Calibrate the camera by a series of images containing a chessboard
 *
 * @param sampleFiles INPUT, filenames of image samples containing chessboard, usually captured by camera
 * @param camMat OUTPUT, estimated camera's intrinsic parameters
 * @param distCoeffs OUPUT, vector of distortion coefficients
 * @param rvecs OUTPUT, vector of rotation vectors
 * @param tvecs OUTPUT, vector of translation vectors estimated
 *
 * @return the final re-projection error.
 *                  
 */
double calibCameraWithChessboard(vector<string>& sampleFiles,  Mat& camMat,
                                Mat& distCoeffs,vector<Mat>& rvecs, vector<Mat>& tvecs)
{
    vector< vector<Point3f> > allObjCorners;
    vector< vector<Point2f> > allImgCorners;

    // sampled image size, in pixels
    Size imgSize=Size(0,0);

    // chessboard size, rows & cols of corners
    Size boardSize=cvSize(CORNER_COLS, CORNER_ROWS);

    // suqare size (side metric length )
    float squareSize=SQUARE_SIZE;
    
    // analyze all input sample images, to get the corners
    for(vector<string>::iterator it=sampleFiles.begin();it!=sampleFiles.end();it++){
        Mat gray;
        Mat image=imread(*it);
        vector<Point2f> imgCorners; // corners coords vector
        
        if(image.data==NULL){
            cerr<<"Failed to open image file '"<<*it<<"', skipping..."<<endl;
            continue;
        }

        // get sampled image size 
        if(!imgSize.area())imgSize=image.size();

        // find corners on chessboard
        if(!findChessboardCorners(image,boardSize,imgCorners,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK |
                CV_CALIB_CB_NORMALIZE_IMAGE)){
            cerr<<"Could not find a complete chessboard, ignoring this image..."<<endl;
            continue;
        }

        // refine the corners' coordinates in sampled image 
        cvtColor(image,gray,CV_BGR2GRAY);
        cornerSubPix(gray, imgCorners,
                    cv::Size(5,5),
                    cv::Size(-1,-1),
                    cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
                                30,      // max number of iterations
                                0.1));   // min accuracy

        // set chessboard-centered corner coords in 3D obj-space
        vector<Point3f> objCorners;
        setChessboardCorners(boardSize, squareSize, objCorners);
        
        // push the corresponding corners(in 3D obj-space & 2D image space) into stack
        allImgCorners.push_back(imgCorners);
        allObjCorners.push_back(objCorners);
    }

    
    //Mat homo=findHomography(allObjCorners[0], allImgCorners[0], CV_RANSAC);
    cout<<cv::Mat(allObjCorners[0]).colRange(0,2)<<endl;
   // cout<<"homo1:\n\t"<<homo<<endl;

    // camera calibration.
    // return the final re-projection error.
    return calibrateCamera(allObjCorners,allImgCorners,imgSize, camMat, distCoeffs, rvecs, tvecs, CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

}

/**
 * Undistort the images, and save them to files
 *
 * @param srcFiles INPUT, filenames of original source image
 * @param dstFiles INPUT, filenames of undistorted image.
 *          If not empty, undistorted images will be saved in these names.
 * @param camMat INPUT, calibrated camera's intrinsic parameters
 * @param distCoeffs INPUT, vector of distortion coefficients
 * @param undistImgs OUTPUT, vector of undistorted images
 *
 * @return 0 on success, others failure
 * */
int undistortImages(vector<string>& srcFiles, vector<string>& dstFiles,
                    Mat& camMat, Mat& distCoeffs, vector<Mat>& undistImgs )
{

    // undistort mapping
    Mat map1,map2;

    // sampled image size, in pixels
    Size imgSize=Size(0,0);
    
    if(!srcFiles.size()){
        cerr<<"Empty image vector!"<<endl;
        return -1;
    }

    vector<string>::iterator sit=srcFiles.begin();
    vector<string>::iterator dit=dstFiles.begin();
    for(;sit!=srcFiles.end(); sit++){
        Mat img=imread(*sit); 
        Mat undist;
        
        if(!img.data){
            cerr<<"Failed to open image file '"<<*sit<<"', skipping..."<<endl;
            continue;
        }

        // in first iteration, calculate the undistort map
        if( !imgSize.area() ){
            imgSize=img.size();
            map1.create(imgSize,CV_32FC1);
            map2.create(imgSize,CV_32FC1);
            undist.create(imgSize,CV_32FC1);
            cv::initUndistortRectifyMap(camMat,distCoeffs,Mat(),Mat(),
                                        imgSize,CV_32FC1,map1,map2);
        }

        remap(img,undist,map1,map2,CV_INTER_LINEAR);
        undistImgs.push_back(undist);

        // save to file, if filename is not empty
        if(dit!=dstFiles.end() && dit->length() ){
            imwrite(*dit,undist);
            dit++;
        }
    }
    
    return 0;
}

/**
 * extract projected points from some images (captured by camera), and store them into a vector
 *
 * @param undistImgs INPUT, vector of undistorted images
 * @param allProjPoints OUTPUT, vector of point-vectors
 *
 * @return 0 on success, others on failure
 */
int extractProjectedPoints(vector<Mat>& undistImgs, vector< vector<Point2f> >& allProjPoints)
{

    Mat kernel=getStructuringElement(MORPH_ELLIPSE, Size(3,3));
    Mat gray,bin;
    for(vector<Mat>::iterator it=undistImgs.begin(); it!=undistImgs.end(); it++){
        cvtColor(*it, gray, CV_BGR2GRAY);
        threshold(gray,bin,210,255,cv::THRESH_BINARY);
        dilate(bin,bin,kernel);

        // 寻找轮廓
        vector< std::vector<cv::Point> > contours;
        findContours(bin,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
        //drawContours(bin,contours,-1,Scalar(255),1);
        
        //cv::Mat heart_img(it->size(),CV_8UC3,cv::Scalar(0,0,0));
        vector<Point2f> projPoints;
        vector< std::vector<cv::Point> >::iterator it2;
        for(it2=contours.begin();it2!=contours.end();it2++){
            Point2f heart=GetHeart(*it2);
            projPoints.push_back(heart);
            //circle(heart_img,heart,2,Scalar(0,0,255),CV_FILLED);
        }
        //imwrite("heart_img.png",heart_img);
        
        // TODO: refine these projPoints to subpixel precision 
        
        // packing up projPoints
        allProjPoints.push_back(projPoints);
    }
}

/**
 * Get rotation matrix, from Rodrigues form to matrix form
 *
 * @param rvecs INPUT, Rodrigues form of rotations
 * @param rmats OUTPUT, matrixs form of rotations
 */
void getRotationMatrix(vector<Mat>& rvecs, vector<Mat>& rmats)
{
    for(vector<Mat>::iterator it=rvecs.begin(); it!=rvecs.end(); it++){
        Mat r;
        Rodrigues(*it,r);
        rmats.push_back(r);
    }
}

/**
 * Get homographies from rotation & translation matrix, and camera intrinsic matrix
 *
 * @param rmats INPUT, rotation matrix
 * @param tvecs INPUT, translation vectors
 * @param camMat INPUT, camera intrinsic matrix
 * @param homos OUTPUT, homographies matrix
 * 
 */
void getHomographyFromRT(vector<Mat>& rmats, vector<Mat>& tvecs, Mat& camMat, vector<Mat>& homos)
{
    for(int i=0; i<rmats.size(); i++){
        Mat mat=cv::Mat(3,3,CV_64F);
        Mat homo=cv::Mat(3,3,CV_64F);
        
        rmats[i].colRange(0,2).copyTo(mat(cv::Rect(0,0,2,3)));
        tvecs[i].copyTo(mat(cv::Rect(2,0,1,3)));
        homo=camMat*mat;
        homos.push_back(homo);
    }

    cout<<"homo2:\n\t"<<homos[0]<<endl;
}

int main()
{
    char fname[32];
    vector<Mat> undistImgs;

    // ---------------------------------------------------------------------------------
    // Calibration procedures
    // ---------------------------------------------------------------------------------
    vector<string> sampleFiles;
    Mat camMat;
    Mat distCoeffs;
    vector<Mat>rvecs,tvecs,rmats;
    vector<Mat>homos;
    
    // packing up sampled images containing chessboard
    for(int i=SNAP_MIN; i<=SNAP_MAX; i++){
        memset(fname,0,sizeof fname);
        sprintf(fname, "snap%02d.png", i);
        sampleFiles.push_back(fname);
    }

    // do camera calibration
    double rms=calibCameraWithChessboard(sampleFiles, camMat, distCoeffs, rvecs, tvecs);
    cout<<" *** The estimated camera intrisic parameters:"<<endl;
    cout<<camMat<<std::endl;
    cout<<"RMS error reported by calibrateCamera:"<<rms<<endl;

    // get roration matrixes
    getRotationMatrix(rvecs, rmats);

    // calculate homographies
    getHomographyFromRT(rmats, tvecs, camMat, homos);

   

    // ---------------------------------------------------------------------------------
    // calculate the undistorted map from camera's intrinsic matrix & distCoeffs
    // ---------------------------------------------------------------------------------
    vector<string> undistFiles;
    for(int i=SNAP_MIN; i<=SNAP_MAX; i++){
        memset(fname,0,sizeof fname);
        sprintf(fname, "undistorted%02d.png", i);
        undistFiles.push_back(fname);
    }
    undistortImages(sampleFiles, undistFiles, camMat, distCoeffs, undistImgs);

    // ---------------------------------------------------------------------------------
    // extract the projected points from undistorted images
    // ---------------------------------------------------------------------------------
    vector< vector<Point2f> > allProjPoints;
    extractProjectedPoints(undistImgs, allProjPoints);
    

    
/*
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

    //-------------------------------------------------------------------------

    cout<<cam_imgPoints.at(1).at(0)<<endl;
    cout<<cam_objPoints.at(1).at(0)<<endl;
    cout<<cameraRT.at(1)<<endl;
*/
    
}
