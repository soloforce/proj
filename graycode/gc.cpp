#include <opencv2/opencv.hpp>
#include <iostream>
#include <bitset>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <unistd.h>
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <pthread.h>
#include <ctime>

#define PROJ_WIDTH 1024
#define PROJ_HEIGHT 768
#define DESKTOP_WIDTH 2560
#define FRAME_INTERVAL 100 // in ms
#define SENSOR_NUMBER   4
#define DX (PROJ_WIDTH/256)
#define DY (PROJ_HEIGHT/256)

#define CALIB_X 'X' // 标定X坐标的指令
#define CALIB_Y 'Y' // 标定Y坐标的指令
#define CALIB_L 'L' // 标定'亮'的指令
#define CALIB_D 'D' // 标定'暗'的指令

#define ERROR_CALIB_TYPE 'E'

// 格雷码的bit数，也就是格雷码结构图像的帧数
#define GC_FRAMES 8

// 坐标的采集遍数，只有满足所有遍数采集到的坐标值都相等了，才认为是稳定的坐标，否则必须重新采集
// 显然，这个数值越大就说明对采集的坐标值的稳定性可靠性要求越高，代价是花的时间也越多; 一般不要大于3，否则可能要花很长时间来标定
// 当该值设置为1时，表示只采集一次
#define COORDS_MULTIPASS 2

#define MAX_BOARD 2

bool board1_exist=true; // big board
bool board2_exist=false; // small board

cv::Mat gcImgH[GC_FRAMES], gcImgV[GC_FRAMES];


std::string window_name="graycode_image";


// 用来存放多遍坐标值的数组
int x_coords[MAX_BOARD][COORDS_MULTIPASS][SENSOR_NUMBER];
int y_coords[MAX_BOARD][COORDS_MULTIPASS][SENSOR_NUMBER];

int x_current_pass[MAX_BOARD]={0};
int y_current_pass[MAX_BOARD]={0};

typedef void*(*GCTHREAD_FUNC)(void *);


static int set_fl(int fd, int flags)
{
    int val;

    val = fcntl(fd, F_GETFL, 0);
    if(val < 0){
        perror("fcntl get error");
        exit(1);
    }

    val |= flags;
    if(fcntl(fd, F_SETFL, val) < 0){
        perror("fcntl set error");
        exit(2);
    }

    return 0;
}

/*
 * @brief Open serial port with the given device name
 *
 * @return The file descriptor on success or -1 on error.
 */
int open_port(char *port_device)
{
    int fd; /* File descriptor for the port */

    fd = open(port_device, O_RDWR  );//O_NOCTTY );
    if (fd == -1)
    {
        perror("open_port: Unable to open port");
        exit(-1);
    }


    return (fd);
}
inline unsigned int bin2gray(unsigned int num)
{
    return (num >> 1) ^ num;
}

unsigned int gray2bin(unsigned int num)
{
    unsigned int mask;
    for (mask = num >> 1; mask != 0; mask = mask >> 1)
    {
        num = num ^ mask;
    }
    return num;
}

void calib(char* buf, int& pipe, char& type)
{
    char* p=strtok(buf,":");
    pipe=strtol(p,NULL,10);
    if(pipe==1) board1_exist=true;
    else if(pipe==2) board2_exist=true;

    p=strtok(NULL,":");
    type=p[0];

    if((type!=CALIB_X) && (type!=CALIB_Y)){
        printf("got an invalid feedback type:%c\n",type);
        return;// ERROR_CALIB_TYPE;
    }

    char* coords=strtok(NULL,":");
    char *s[SENSOR_NUMBER];

    s[0]=strtok(coords,",");
    int i;
    for(i=1;i<SENSOR_NUMBER;i++) s[i]=strtok(NULL,",");

    if(type==CALIB_X) for(i=0;i<SENSOR_NUMBER;i++){
        if(x_current_pass[pipe-1]<COORDS_MULTIPASS)
            x_coords[pipe-1][x_current_pass[pipe-1]][i]=gray2bin(strtol(s[i],NULL,16));
        //printf("%d\n",x_coords[i]);
    }else if(type==CALIB_Y) for(i=0;i<SENSOR_NUMBER;i++){
        if(y_current_pass[pipe-1]<COORDS_MULTIPASS)
            y_coords[pipe-1][y_current_pass[pipe-1]][i]=gray2bin(strtol(s[i],NULL,16));
        //printf("%d\n",y_coords[i]);
    }

}

bool is_calib_stable(int (*coords)[4], int current_pass)
{
    if(!coords) return false;

    // 第一遍，假定是稳定的
    if(current_pass==0 || current_pass>= COORDS_MULTIPASS) return true;

    // 和上一遍相比,若有一个坐标相差太大，则认为这几遍是不稳定的；
    for(int i=0; i<SENSOR_NUMBER; i++){
        //if( coords[current_pass-1][i]!=coords[current_pass][i] ) return false;
        if( abs(coords[current_pass-1][i]-coords[current_pass][i] ) > 3) return false;
    }

    // 该遍通过稳定性测试;
    return true;
}

void check_stability(int pipe, char type)
{
    bool b;

    switch(type){
    case ERROR_CALIB_TYPE:
        std::cout<<"Invalid calib type!"<<std::endl;
        return;
    case CALIB_X:
        if(x_current_pass[pipe-1]>=COORDS_MULTIPASS)return;
        b=is_calib_stable(x_coords[pipe-1], x_current_pass[pipe-1]);
        if(b){
            // 当前遍的稳定性通过测试
            std::cout<<"X-calibration stability of pass " <<x_current_pass[pipe-1]<<" is ok!"<<std::endl;
            x_current_pass[pipe-1]++;

        }else{
            // 稳定性不足，推倒重来
            std::cout<<"X-calibration stability of pass " <<x_current_pass[pipe-1]<<" is not good, retry from the beginning!"<<std::endl;
            x_current_pass[pipe-1]=0;
        }
        break;
    case CALIB_Y:
        if(y_current_pass[pipe-1]>=COORDS_MULTIPASS) return;

        b=is_calib_stable(y_coords[pipe-1], y_current_pass[pipe-1]);
        if(b){
            // 当前遍的稳定性通过测试
            std::cout<<"Y-calibration stability of pass " <<y_current_pass[pipe-1]<<" is ok!"<<std::endl;
            y_current_pass[pipe-1]++;

        }else{
            // 稳定性不足，推倒重来
            std::cout<<"Y-calibration stability of pass " <<y_current_pass[pipe-1]<<" is not good, retry from the beginning!"<<std::endl;
            y_current_pass[pipe-1]=0;
        }
        break;
    default:
        std::cout<<"Impossible type!"<<std::endl;
        return;
    }

}

void* get_result( void *ptr )
{
    char buf[1024];
    int fd=*((int*)ptr);
    char* pos;
    int n;
    int pipe;
    char type;

    pos=&buf[0];
    while(1){
        if((n=read(fd, pos, 1))==1 ){
            if(*pos=='\n'){
                *pos=0;

                if(pos!=buf){
                    //接收到一个反馈
                    printf("%s\n",buf);
                    fflush(stdout);
                    calib(buf,pipe,type);
                    check_stability(pipe,type);
                }

                pos=&buf[0];
                continue;
            }
            pos++;
        }
    }

}



void generate_graycode(int frames, int *codes)
{
    if(!codes) return;
    for(int i=0; i<pow(2,frames); i++){
        codes[i]=(i>>1)^i;
        //std::cout<<i<<":"<<std::bitset<sizeof(char)*8>(codes[i])<<std::endl;
    }
}




cv::Rect getBoundingRect(std::vector<cv::Point2d>& points)
{
    cv::Rect boundingRect;
    double minR,maxR, minC, maxC;
    for(std::vector<cv::Point2d>::iterator it=points.begin(); it!=points.end(); it++){
        if(it==points.begin()){
            minC=it->x;
            maxC=it->x;
            minR=it->y;
            maxR=it->y;
        }else{
            if(minC > it->x) minC = it->x;
            if(maxC < it->x) maxC = it->x;
            if(minR > it->y) minR = it->y;
            if(maxR < it->y) maxR = it->y;
        }
    }

    boundingRect.x=minC;
    boundingRect.y=minR;
    boundingRect.width=maxC-minC;
    boundingRect.height=maxR-minR;

    return boundingRect;
}


std::vector<cv::Point2d> centerPoints(std::vector<cv::Point2d>& points, cv::Point2d oldCenter, cv::Point2d newCenter)
{
    std::vector<cv::Point2d> cpts;
    cv::Point2d pt;
    float xoffset=newCenter.x-oldCenter.x;
    float yoffset=newCenter.y-oldCenter.y;
    for(std::vector<cv::Point2d>::iterator it=points.begin(); it!=points.end(); it++){
        pt.x=it->x + xoffset;
        pt.y=it->y + yoffset;
        cpts.push_back(pt);
    }
    return cpts;
}


cv::Rect scaleRect(cv::Rect srt, float scale)
{
    cv::Rect drt;
    double cx=srt.x+srt.width/2.0;
    double cy=srt.y+srt.height/2.0;

    drt.x=cx-scale*srt.width/2.0;
    drt.y=cy-scale*srt.height/2.0;
    drt.width=scale*srt.width;
    drt.height=scale*srt.height;

    //std::cout<<"!!\n"<<drt<<std::endl;
    return drt;
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


void getHomoPoints(std::vector<cv::Point3f>& hsrc, std::vector<cv::Point3f>& hdes, cv::Mat H, float scale=1.0)
{
    cv::Point3f pt;

    if(!H.data) return;
    if(!scale) H.at<double_t>(2,2)=scale;
    for(std::vector<cv::Point3f>::iterator it=hsrc.begin(); it!=hsrc.end(); it++){
       // pt=H.inv().mul(*it);
        hdes.push_back(pt);
    }
}

std::vector<cv::Point2d> rectToPoints(cv::Rect rect)
{
    std::vector<cv::Point2d> pts;
    cv::Point2d pt;
    pt.x=rect.x;
    pt.y=rect.y;
    pts.push_back(pt);
    pt.x=rect.x+rect.width;
    pt.y=rect.y;
    pts.push_back(pt);
    pt.x=rect.x+rect.width;
    pt.y=rect.y+rect.height;
    pts.push_back(pt);
    pt.x=rect.x;
    pt.y=rect.y+rect.height;
    pts.push_back(pt);

    return pts;
}


bool isPointsInRect(std::vector<cv::Point2d>& pts, cv::Rect rect)
{
    for(std::vector<cv::Point2d>::iterator it=pts.begin(); it!=pts.end(); it++){
        if(!rect.contains(*it)) return false;
    }
    return true;
}


cv::Mat generate_perspective_image(int pipe)
{

    std::vector<cv::Point2d> src_points;
    std::vector<cv::Point2d> dst_points;

    src_points.push_back(cv::Point2d(0,0));
    src_points.push_back(cv::Point2d(1023,0));
    src_points.push_back(cv::Point2d(1023,767));
    src_points.push_back(cv::Point2d(0,767));

    for(int i=0; i<SENSOR_NUMBER; i++){
        dst_points.push_back(cv::Point2d(x_coords[pipe-1][0][i]*DX, y_coords[pipe-1][0][i]*DY));
    }


    cv::Mat per=findHomography(src_points, dst_points);

    cv::Mat srcImg,warpImg;
    if(pipe==1){
        srcImg=cv::imread("src1.png");//cv::imread("art_girl.png");
    }else srcImg=cv::imread("lady_kesake.png");

    cv::warpPerspective(srcImg, warpImg, per, srcImg.size());


    return warpImg;
}

void output_perspective_images(cv::Mat warp1, cv::Mat warp2)
{
    struct tm *local;
    char filename[256];
    time_t t;
    t=time(NULL);
    local=localtime(&t);

    sprintf(filename, "fit_warped_%d%d%d%d%d%d.png",local->tm_year, local->tm_mon, local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec);


    std::string window_name="warped";
    cv::namedWindow(window_name);

    // 移动到第二个屏幕上（第一个屏幕的大小是2560x1440)
    cv::moveWindow(window_name,DESKTOP_WIDTH,0);

    // 全屏显示窗口。事实上未必会全屏，但至少可以去掉标题栏部分
    cv::setWindowProperty(window_name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    cv::Mat warpImg;
    warp1.copyTo(warpImg);
    warp2.copyTo(warpImg,warp2);

    cv::imwrite(filename, warpImg);
    cv::imshow(window_name, warpImg);
}


void calib_X(int fd)
{
    // 开始标定坐标
    std::cout<<"Now begin to calibrate X coordinates..."<<std::endl;
    // 发送X标定指令到采集端
    //std::cout<<"Ready to send a X calibration instruction..."<<std::endl;
    char wbuf[1]={CALIB_X};
    size_t n=write(fd,wbuf, 1);
    if(n!=1){
        std::cout<<"Write ttyUSB error, retrying..."<<std::endl;
       // continue; // 发送失败，尝试重发
    }

    // 开始投影格雷码结构图序列
    for(int i=0; i<GC_FRAMES; i++){
        cv::imshow(window_name, gcImgH[i]);
        cv::waitKey(FRAME_INTERVAL);
    }

    // 做适当的延迟，以等待工作线程接收采集端的反馈信息, 并对信息做检查
    std::cout<<"Waiting for X-calibration feedback..."<<std::endl;
    usleep(500000);
}

void calib_Y(int fd)
{
    std::cout<<"Now calib Y coordinates..."<<std::endl;
    // 发送Y标定指令到采集端
    std::cout<<"Ready to send a Y calibration instruction..."<<std::endl;
    char wbuf[1]={CALIB_Y};
    size_t n=write(fd, wbuf, 1);
    if(n!=1){
        std::cout<<"Write ttyUSB error, retrying..."<<std::endl;
        //continue; // 发送失败，尝试重发
    }

    // 开始投影格雷码结构图序列
    for(int i=0; i<GC_FRAMES; i++){
        cv::imshow(window_name, gcImgV[i]);
        cv::waitKey(FRAME_INTERVAL);
    }

    // 做适当的延迟，以等待工作线程接收采集端的反馈信息, 并对信息做检查
    std::cout<<"Waiting for Y-calibration feedback..."<<std::endl;
    usleep(500000);
}

int main(int argc, char* argv[])
{

    int n=pow(2,GC_FRAMES);
    int *codes=new int[n];

    struct termios options;
    pthread_t get_result_thread;
    void * thread_ret;

    if(argc<3){
        printf("usage:\n\t%s <dev_file> <baud rate>\n",argv[0]);
        return -1;
    }
    printf("Selected baud rate is %ld\n",atol(argv[2]));

    int fd=open_port(argv[1]);
    if(fd==-1){
        return -1;
    }

    tcgetattr(fd, &options);


    //Set the baud rate
    switch(atol(argv[2])){
    case 9600:
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);
        break;
    case 19200:
        cfsetispeed(&options, B19200);
        cfsetospeed(&options, B19200);
        break;
    case 38400:
        cfsetispeed(&options, B38400);
        cfsetospeed(&options, B38400);
        break;
    case 57600:
        cfsetispeed(&options, B57600);
        cfsetospeed(&options, B57600);
        break;
    case 115200:
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        break;
    case 230400:
        cfsetispeed(&options, B230400);
        cfsetospeed(&options, B230400);
        break;
    default:
        printf("Selected baud rate %ld is not support now!\n", atol(argv[2]));
        close(fd);
        return -1;
    }


    //Enable the receiver and set local mode...
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE; /* Mask the character size bits */
    options.c_cflag |= CS8;    /* Select 8 data bits */

    //No parity
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;


    //Set the new options for the port...
    tcsetattr(fd, TCSANOW, &options);
    GCTHREAD_FUNC gc_func=get_result;
    pthread_create( &get_result_thread, NULL, gc_func, (void*)&fd);



    //------------------------------------------------------------------------
    generate_graycode(GC_FRAMES,codes);


    cv::Scalar color;


    cv::namedWindow(window_name);

    // 移动到第二个屏幕上（第一个屏幕的大小是2560x1440)
    cv::moveWindow(window_name,DESKTOP_WIDTH,0);

    // 全屏显示窗口。事实上未必会全屏，但至少可以去掉标题栏部分
    cv::setWindowProperty(window_name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);


    // show an image
    cv::Mat mengnalisha=cv::imread("src1.png");
    cv::imshow(window_name, mengnalisha);
    cv::waitKey(0);

    //------------------------------------------------------------------------
    for(int i=0; i<GC_FRAMES; i++){
        gcImgH[i].create(PROJ_HEIGHT,PROJ_WIDTH,CV_8UC3);
        for(int c=0; c<n; c++){
            int delta_c=PROJ_WIDTH/n;
            int b=(codes[c]>>(GC_FRAMES-1-i)) & 0x01;
            if(!b) color=cv::Scalar(0,0,0);
            else color=cv::Scalar(255,255,255);

            cv::rectangle(gcImgH[i], cv::Point(c*delta_c,0), cv::Point((c+1)*delta_c,PROJ_HEIGHT-1), color, cv::FILLED);
        }

        gcImgV[i].create(PROJ_HEIGHT,PROJ_WIDTH,CV_8UC3);
        for(int r=0; r<n; r++){
            int delta_r=PROJ_HEIGHT/n;
            int b=(codes[r]>>(GC_FRAMES-1-i)) & 0x01;
            if(!b) color=cv::Scalar(0,0,0);
            else color=cv::Scalar(255,255,255);

            cv::rectangle(gcImgV[i], cv::Point(0,r*delta_r), cv::Point(PROJ_WIDTH-1, (r+1)*delta_r), color, cv::FILLED);
        }

        // 保存两种格雷码结构图
        /*
        char fname[32];
        snprintf(fname,32,"gcImgV%02d.png", i);
        std::cout<<fname<<std::endl;
        cv::imwrite(fname, gcImgV[i]);
        snprintf(fname,32,"gcImgH%02d.png", i);
        std::cout<<fname<<std::endl;
        cv::imwrite(fname, gcImgH[i]);
        */
    }

    cv::Mat black(cv::Size(PROJ_WIDTH,PROJ_HEIGHT), CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat white(cv::Size(PROJ_WIDTH,PROJ_HEIGHT), CV_8UC3, cv::Scalar(255,255,255));


    // 标定亮暗水平
    std::cout<<"Prepare step: begin to detect light level, wait for some seconds..."<<std::endl;
    char wbuf[1];
    wbuf[0]=CALIB_L;
    n=write(fd,wbuf, 1);
    if(n!=1){
        std::cout<<"Write ttyUSB error!"<<std::endl;
    }
    cv::imshow(window_name, white);
    cv::waitKey(500);


    std::cout<<"Prepare step: begin to detect dark level, wait for some seconds..."<<std::endl;
    wbuf[0]=CALIB_D;
    n=write(fd,wbuf, 1);
    if(n!=1){
        std::cout<<"Write ttyUSB error!"<<std::endl;
    }
    cv::imshow(window_name, black);
    cv::waitKey(500);

    if((!board1_exist) && !(board2_exist)){
        std::cout<<"No projecting board found!"<<std::endl;
        return -1;
    }

    if(board1_exist){
        while(x_current_pass[0]<COORDS_MULTIPASS){
            calib_X(fd);
        }

        while(y_current_pass[0]<COORDS_MULTIPASS){
            calib_Y(fd);
        }
    }

    if(board2_exist){
        while(x_current_pass[1]<COORDS_MULTIPASS){
            calib_X(fd);
        }
        while(y_current_pass[1]<COORDS_MULTIPASS){
            calib_Y(fd);
        }
    }

    cv::Mat warp1(cv::Size(PROJ_WIDTH,PROJ_HEIGHT), CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat warp2(cv::Size(PROJ_WIDTH,PROJ_HEIGHT), CV_8UC3, cv::Scalar(0,0,0));

    if(board1_exist){
        std::cout<<"X & Y coordinates are stable, now to calibrate the projector..."<<std::endl;
        // 输出坐标值
        for(int i=0;i<SENSOR_NUMBER;i++){
            std::cout<<"("<<x_coords[0][0][i]<<","<<y_coords[0][0][i]<<")"<<std::endl;
        }

        warp1=generate_perspective_image(1);

    }

    if(board2_exist){
        std::cout<<"X & Y coordinates are stable, now to calibrate the projector..."<<std::endl;
        // 输出坐标值
        for(int i=0;i<SENSOR_NUMBER;i++){
            std::cout<<"("<<x_coords[1][0][i]<<","<<y_coords[1][0][i]<<")"<<std::endl;
        }

        warp2=generate_perspective_image(2);
    }

    output_perspective_images(warp1,warp2);
    cv::waitKey(0);

    // 等待接收线程结束
    pthread_join(get_result_thread,&thread_ret);

}
