#include <opencv2/opencv.hpp>
#include <X11/Xlib.h>
#include <xdo.h>
#include <signal.h>

#define OPENCV3

#define TARGET_SCREEN 1
#define TARGET_PROJECTOR 0

const int CALIB_POINTS_COUNT=4;

using namespace std;
using namespace cv;

typedef enum{
    MOUSE_NA=0,
    MOUSE_DOWN,
    MOUSE_UP,
    MOUSE_MOVE
}mouse_state;

bool running=true;

VideoCapture camera;

xdo_t *xdo;

vector<Point2d> calibCameraPoints;

struct sigaction int_act,term_act, old_act;

typedef struct{
    long int user,nice,sys,idle,iowait,irq,softirq;
}jiffies_t;



void xdo_init()
{
    xdo=xdo_new(NULL);
}

void xdo_cleanup()
{
    xdo_free(xdo);
}

/**
* @brief Signal INT handler
* */
void sigint_handler(int sig_num)
{
    cout<<"Exiting..."<<endl;


    camera.release();
    running=false;
    xdo_cleanup();

    /* change back to the original handler */
    sigaction(SIGINT, &old_act,NULL);


    /* emit the SIGINT again */
    kill(0,SIGINT);
}

/**
* @brief Signal TERM handler
* */
void sigterm_handler(int sig_num)
{
    cout<<"Exiting..."<<endl;

    camera.release();
    running=false;
    xdo_cleanup();

    /* change back to the original handler */
    sigaction(SIGTERM, &old_act,NULL);

    /* emit the SIGTERM again */
    kill(0,SIGTERM);
}


/**
* @brief Initialize signals
* * */
void catch_signal_init()
{
    memset(&int_act,0,sizeof(int_act));
    memset(&term_act,0,sizeof(int_act));
    memset(&old_act,0,sizeof(old_act));
    int_act.sa_handler=&sigint_handler;
    term_act.sa_handler=&sigterm_handler;
    sigaction(SIGINT, &int_act, &old_act);
    sigaction(SIGTERM, &term_act, &old_act);
}



Point2d getHeart(vector<Point>& v)
{
    Point2d pt(0,0);
    int n=0;

    vector<Point>::iterator it;
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

int getLargestContour(vector< vector<Point> >& contours)
{

    int len, pos;
    if(contours.size()){
        len=contours[0].size();
        pos=0;
    }

    for(int i=0; i<contours.size(); i++){
        if(contours[i].size()>len){
            len=contours[i].size();
            pos=i;
        }
    }

    return pos;
}

void getScreenResolution(int* width, int* height)
{
    const char *command="xrandr | grep '*'";
    FILE *fpipe = (FILE*)popen(command,"r");
    char line[256];
    char *w, *h;

    if( fgets( line, sizeof(line), fpipe)){
        w=strtok(line, "x\t ");
        h=strtok(NULL, "x\t ");
        *width=atoi(w);
        *height=atoi(h);
    }
    pclose(fpipe);
}

void getScreenResolution2(int* width, int* height)
{
    int screen_num;
    Display *display;
    XEvent event;

    display=XOpenDisplay(":0");
    if (display==NULL) {
        cerr<<"Cannot connect to X server %s\n :0"<<endl;
        return;
    }

    screen_num = DefaultScreen(display);
    *width=DisplayWidth(display,screen_num);
    *height=DisplayHeight(display,screen_num);

    XCloseDisplay(display);

}

// send a mouse-click event with xdotool
void mouseAction(mouse_state mst, int x, int y)
{


    switch(mst){
    case MOUSE_DOWN:
        xdo_move_mouse(xdo, x, y, 0);
        xdo_mouse_down(xdo, CURRENTWINDOW, 1);
        cout<<"MouseDown at "<<Point(x,y)<<endl;
        break;
    case MOUSE_MOVE:
        xdo_move_mouse(xdo, x, y, 0);
        cout<<"MouseMove at "<<Point(x,y)<<endl;
        break;
    case MOUSE_UP:
        xdo_mouse_up(xdo, CURRENTWINDOW, 1);
        cout<<"MouseUp at "<<Point(x,y)<<endl;
        break;
    }

}

// send a mouse-click event with xdotool
void mouseAction2(mouse_state mst, int x, int y)
{
    char cmd[256]={0};
    switch(mst){
    case MOUSE_DOWN:
        snprintf(cmd, sizeof cmd, "xdotool mousemove %d %d mousedown 1", x, y);
        cout<<"MouseDown at "<<Point(x,y)<<endl;
        break;
    case MOUSE_MOVE:
        snprintf(cmd, sizeof cmd, "xdotool mousemove %d %d", x, y);
        cout<<"MouseMove at "<<Point(x,y)<<endl;
        break;
    case MOUSE_UP:
        snprintf(cmd, sizeof cmd, "xdotool mouseup 1");
        cout<<"MouseUp at "<<Point(x,y)<<endl;
        break;
    }

    if(cmd[0]) system(cmd);
}

void drawCalibGrids()
{
    //int screen_width, screen_height;
    //getScreenResolution2(&screen_width, &screen_height);
    //cout<<"Screen resolution: "<<screen_width<<"x"<<screen_height<<endl;
}


cv::Mat pointsToMat(std::vector<cv::Point2d>& pts)
{
    // each column of hm is a homogeneous coordinate of a point.
    int c=pts.size(); // amount of points
    cv::Mat hm(3,c,CV_64FC1,cv::Scalar(1.0));

    cv::Mat m(pts);
    m=m.reshape(1,c);
    m=m.t();
    cv::Mat roi(hm, cv::Rect(0,0,c,2));
    m.copyTo(roi);

    return hm;
}

std::vector<cv::Point2d> matToPoints(cv::Mat m)
{
    std::vector<cv::Point2d> pts;
    cv::Point2d pt;

    if(m.rows==3) m=m.t();
    int rows=m.rows;
    for(int r=0; r<rows; r++){
        pt.x=m.at<double_t>(r,0)/m.at<double_t>(r,2);
        pt.y=m.at<double_t>(r,1)/m.at<double_t>(r,2);
        pts.push_back(pt);
    }

    return pts;
}

// convert a point to a homogeneous coordinate
Mat point2mat(Point2d pt)
{
    Mat hm(3,1,CV_64FC1,cv::Scalar(1.0));
    hm.at<double_t>(0,0)=pt.x;
    hm.at<double_t>(1,0)=pt.y;
    return hm;
}

Point2d mat2point(Mat m)
{
    Point2d pt(0,0);
    double_t d=m.at<double_t>(2,0);
    if(d){
        pt.x=m.at<double_t>(0,0)/d;
        pt.y=m.at<double_t>(1,0)/d;
    }
    return pt;
}

void calibrate(vector<Point2d> cameraPts, Mat& homo)
{
    vector<Point2d> screenPts;
    int screen_width, screen_height;
    if(TARGET_SCREEN ) getScreenResolution2(&screen_width, &screen_height);
    else if(TARGET_PROJECTOR){
        screen_width=1024;
        screen_height=768;
    }

    screenPts.push_back(Point2d(0,0));
    screenPts.push_back(Point2d(screen_width-1, 0));
    screenPts.push_back(Point2d(screen_width-1, screen_height-1));
    screenPts.push_back(Point2d(0,screen_height-1));

    homo=findHomography(cameraPts, screenPts);

}

// sort the vectors by descending size
bool vector_size_comp(vector<Point> v1, vector<Point> v2)
{
    return (v1.size() > v2.size() );
}


int main(int argc, char** argv)
{

    catch_signal_init();
    xdo_init();

    FileStorage fs("camera.xml", FileStorage::READ);

    if(!fs.isOpened()){
        cout<<"Failed to open the file storing camera's intrinsic parameters!"<<endl;
        return -1;
    }

    Mat cameraMatrix, distCoeffs;
    Mat map1, map2;
    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Distortion_Coefficients"] >> distCoeffs;

    if(!cameraMatrix.data || !distCoeffs.data){
        cout<<"Failed to load camera's intrinsic parameters!"<<endl;
        fs.release();
        return -1;
    }



    int cameraNumber=0;
    if(argc>1) cameraNumber=atoi(argv[1]);

    Rect screenRect(0,0,0,0);
    if(TARGET_SCREEN) getScreenResolution2(&screenRect.width, &screenRect.height);
    else if(TARGET_PROJECTOR) { screenRect.width=1024; screenRect.height=768; }


    camera.open(cameraNumber);
    if(!camera.isOpened()){
        cerr<<"ERROR: Could not open camera!"<<endl;
        exit(1);
    }


    Point2d ptTouched(-1,-1);
    Point2d ptMapped(-1,-1);
    bool isCalibrated=false;
    bool isFirstFrame=true;

    Mat homo;

    mouse_state mst=MOUSE_NA;

    bool once=true;

    while(running){
        Mat oriframe, frame, gray;

        camera>>oriframe;
        if(oriframe.empty()){
            cerr<<"ERROR: Could not grab a frame!"<<endl;
            exit(1);
        }

        if(isFirstFrame){
            isFirstFrame=false;
            map1.create(oriframe.size(),CV_32FC1);
            map2.create(oriframe.size(),CV_32FC1);
            frame.create(oriframe.size(),CV_32FC1);
            cv::initUndistortRectifyMap(cameraMatrix,distCoeffs,cv::Mat(),
                            cv::Mat(),frame.size(),CV_32FC1,map1,map2);
        }

        double t=getTickCount();

        #ifdef OPENCV3
            cv::remap(oriframe,frame,map1,map2,cv::INTER_LINEAR);
        #else
            cv::remap(oriframe,frame,map1,map2,CV_INTER_LINEAR);
        #endif


        cvtColor(frame, gray, COLOR_BGR2GRAY);

        //const int MEDIAN_BLUR_FILTER_SIZE=7;
        //medianBlur(gray, gray, MEDIAN_BLUR_FILTER_SIZE);

        threshold(gray, gray, 200, 255, THRESH_BINARY);

        vector< vector<Point> > contours;

        // 寻找轮廓会修改gray本身，因此用gray.clone
        // 如果gray不再被使用，那就不用clone方法，而是可以直接用gray本身了
        findContours(gray.clone(),contours,RETR_EXTERNAL,CHAIN_APPROX_NONE);


        if(contours.size()){
            if(mst==MOUSE_DOWN) mst=MOUSE_MOVE;
            else if(mst==MOUSE_NA || mst==MOUSE_UP) mst=MOUSE_DOWN;

            sort(contours.begin(), contours.end(), vector_size_comp);
            ptTouched=getHeart(contours[0]);
            circle(frame, ptTouched, 2, Scalar(0,255,0), 2, LINE_AA);

        }else{
            if(mst==MOUSE_UP) mst=MOUSE_NA;
            else if( (mst==MOUSE_DOWN) || (mst==MOUSE_MOVE) ) mst=MOUSE_UP;

            if((!isCalibrated) && Rect(0,0,frame.cols,frame.rows).contains(ptTouched)){
                if(calibCameraPoints.size()<CALIB_POINTS_COUNT){
                    calibCameraPoints.push_back(ptTouched);
                }

                if(calibCameraPoints.size()>=CALIB_POINTS_COUNT){
                    calibrate(calibCameraPoints, homo);
                    isCalibrated=true;
                }

                // clear the touched point
                ptTouched.x=ptTouched.y=-1;

            }
        }

        // Draw the calibration points on camera frames.
        for(int i=0; i<calibCameraPoints.size(); i++){
            circle(frame, calibCameraPoints[i], 3, Scalar(0,0,255), 2, LINE_AA);
        }

        // Draw the polygon formed by the calibCameraPoints
        if(isCalibrated){
            line(frame, calibCameraPoints[0], calibCameraPoints[1], Scalar(0,255,0), 1, LINE_AA);
            line(frame, calibCameraPoints[1], calibCameraPoints[2], Scalar(0,255,0), 1, LINE_AA);
            line(frame, calibCameraPoints[2], calibCameraPoints[3], Scalar(0,255,0), 1, LINE_AA);
            line(frame, calibCameraPoints[3], calibCameraPoints[0], Scalar(0,255,0), 1, LINE_AA);
        }

        if(isCalibrated && Rect(0,0,frame.cols,frame.rows).contains(ptTouched)){

            Mat m=point2mat(ptTouched);
            ptMapped=mat2point(homo*m);
            if(screenRect.contains(ptMapped)){
                mouseAction(mst, ptMapped.x, ptMapped.y);
                if(mst==MOUSE_UP){
                    mst=MOUSE_NA;
                    // clear the touched point
                    ptTouched.x=ptTouched.y=-1;
                }
            }

        }

        t=getTickCount() -t;
        cout<<"Run time="<<t/(getTickFrequency())*1000<<"ms"<<endl;

        //imshow("gray", gray);
        imshow("camera", frame);

        char key=waitKey(10);
        if(key==27)break; // ESC key to quit loop

        switch(key){
        case 'r':
            isCalibrated=false;
            calibCameraPoints.clear();
            break;
        case 'c':
            isCalibrated=false;
            if(!calibCameraPoints.empty())calibCameraPoints.pop_back();
            break;
        }
    }

    camera.release();
    xdo_cleanup();
}
