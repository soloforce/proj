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
// 显然，这个数值越大就说明对采集的坐标值的稳定性可靠性要求越高，代价是花的时间也越多; 一般不要大于3
// 当该值设置为1时，表示只采集一次
#define COORDS_MULTIPASS 2



// 用来存放多遍坐标值的数组
int x_coords[COORDS_MULTIPASS][SENSOR_NUMBER];
int y_coords[COORDS_MULTIPASS][SENSOR_NUMBER];

int x_current_pass=0;
int y_current_pass=0;

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

char calib(char* buf)
{
    char type;
    int pipe;

    char* p=strtok(buf,":");
    pipe=strtol(p,NULL,10);
    p=strtok(NULL,":");
    type=p[0];

    if((type!=CALIB_X) && (type!=CALIB_Y)){
        printf("got an invalid feedback type:%c\n",type);
        return ERROR_CALIB_TYPE;
    }

    char* coords=strtok(NULL,":");
    char *s[SENSOR_NUMBER];

    s[0]=strtok(coords,",");
    int i;
    for(i=1;i<SENSOR_NUMBER;i++) s[i]=strtok(NULL,",");

    if(type==CALIB_X) for(i=0;i<SENSOR_NUMBER;i++){
        x_coords[x_current_pass][i]=gray2bin(strtol(s[i],NULL,16));
        //printf("%d\n",x_coords[i]);
    }else if(type==CALIB_Y) for(i=0;i<SENSOR_NUMBER;i++){
        y_coords[y_current_pass][i]=gray2bin(strtol(s[i],NULL,16));
        //printf("%d\n",y_coords[i]);
    }


    return type;
}

bool calib_is_stable(int (*coords)[4], int current_pass)
{
    if(!coords) return false;

    // 第一遍，假定是稳定的
    if(current_pass==0) return true;

    // 和上一遍相比,若有一个坐标不同，则认为这几遍是不稳定的；
    for(int i=0; i<SENSOR_NUMBER; i++){
        if( coords[current_pass-1][i]!=coords[current_pass][i] ) return false;
    }

    // 若完全相同，则该遍通过稳定性测试;
    return true;
}

void check_stability(char type)
{
    bool b;

    switch(type){
    case ERROR_CALIB_TYPE:
        std::cout<<"Invalid calib type!"<<std::endl;
        return;
    case CALIB_X:
        b=calib_is_stable(x_coords, x_current_pass);
        if(b){
            // 当前遍的稳定性通过测试
            std::cout<<"X-calibration stability of pass " <<x_current_pass<<" is ok!"<<std::endl;
            x_current_pass++;
        }else{
            // 稳定性不足，推倒重来
            std::cout<<"X-calibration stability of pass " <<x_current_pass<<" is not good, retry from the beginning!"<<std::endl;
            x_current_pass=0;
        }
        break;
    case CALIB_Y:
        b=calib_is_stable(y_coords, y_current_pass);
        if(b){
            // 当前遍的稳定性通过测试
            std::cout<<"Y-calibration stability of pass " <<y_current_pass<<" is ok!"<<std::endl;
            y_current_pass++;
        }else{
            // 稳定性不足，推倒重来
            std::cout<<"Y-calibration stability of pass " <<y_current_pass<<" is not good, retry from the beginning!"<<std::endl;
            y_current_pass=0;
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
                    check_stability(calib(buf));
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
        std::cout<<i<<":"<<std::bitset<sizeof(char)*8>(codes[i])<<std::endl;
    }
}

void output_perspective_image()
{
    std::vector<cv::Point2f> src_points;
    std::vector<cv::Point2f> dst_points;

    src_points.push_back(cv::Point2f(0,0));
    src_points.push_back(cv::Point2f(1023,0));
    src_points.push_back(cv::Point2f(1023,767));
    src_points.push_back(cv::Point2f(0,767));

    for(int i=0; i<SENSOR_NUMBER; i++){
        dst_points.push_back(cv::Point2f(x_coords[0][i]*DX, y_coords[0][i]*DY));
    }


    cv::Mat per=getPerspectiveTransform(src_points, dst_points);

    cv::Mat srcImg=cv::imread("src.png");
    cv::Mat warpImg;
    cv::warpPerspective(srcImg, warpImg, per, srcImg.size());

    cv::imwrite("warped.png", warpImg);

    cv::string window_name="warped";
    cv::namedWindow(window_name);

    // 移动到第二个屏幕上（第一个屏幕的大小是2560x1440)
    cv::moveWindow(window_name,DESKTOP_WIDTH,0);

    // 全屏显示窗口。事实上未必会全屏，但至少可以去掉标题栏部分
    cv::setWindowProperty(window_name, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

    cv::imshow(window_name, warpImg);

    cv::waitKey(0);
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

    cv::Mat gcImgH[GC_FRAMES], gcImgV[GC_FRAMES];
    cv::Scalar color;

    cv::string window_name="graycode_image";
    cv::namedWindow(window_name);

    // 移动到第二个屏幕上（第一个屏幕的大小是2560x1440)
    cv::moveWindow(window_name,DESKTOP_WIDTH,0);

    // 全屏显示窗口。事实上未必会全屏，但至少可以去掉标题栏部分
    cv::setWindowProperty(window_name, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

    for(int i=0; i<GC_FRAMES; i++){
        gcImgH[i].create(PROJ_HEIGHT,PROJ_WIDTH,CV_8UC3);
        for(int c=0; c<n; c++){
            int delta_c=PROJ_WIDTH/n;
            int b=(codes[c]>>(GC_FRAMES-1-i)) & 0x01;
            if(!b) color=cv::Scalar(0,0,0);
            else color=cv::Scalar(255,255,255);

            cv::rectangle(gcImgH[i], cv::Point(c*delta_c,0), cv::Point((c+1)*delta_c,PROJ_HEIGHT-1), color, CV_FILLED);
        }

        gcImgV[i].create(PROJ_HEIGHT,PROJ_WIDTH,CV_8UC3);
        for(int r=0; r<n; r++){
            int delta_r=PROJ_HEIGHT/n;
            int b=(codes[r]>>(GC_FRAMES-1-i)) & 0x01;
            if(!b) color=cv::Scalar(0,0,0);
            else color=cv::Scalar(255,255,255);

            cv::rectangle(gcImgV[i], cv::Point(0,r*delta_r), cv::Point(PROJ_WIDTH-1, (r+1)*delta_r), color, CV_FILLED);
        }

        // 保存两种格雷码结构图
        char fname[32];
        snprintf(fname,32,"gcImgV%02d.png", i);
        std::cout<<fname<<std::endl;
        cv::imwrite(fname, gcImgV[i]);
        snprintf(fname,32,"gcImgH%02d.png", i);
        std::cout<<fname<<std::endl;
        cv::imwrite(fname, gcImgH[i]);
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

    // 开始标定坐标
    std::cout<<"Now begin to calibrate X coordinates..."<<std::endl;
    while(x_current_pass<COORDS_MULTIPASS){
        // 发送X标定指令到采集端
        std::cout<<"Ready to send a X calibration instruction..."<<std::endl;
        char wbuf[1]={CALIB_X};
        size_t n=write(fd,wbuf, 1);
        if(n!=1){
            std::cout<<"Write ttyUSB error, retrying..."<<std::endl;
            continue; // 发送失败，尝试重发
        }

        // 开始投影格雷码结构图序列
        for(int i=0; i<GC_FRAMES; i++){
            cv::imshow(window_name, gcImgH[i]);
            cv::waitKey(FRAME_INTERVAL);
        }

        // 做适当的延迟，以等待工作线程接收采集端的反馈信息, 并对信息做检查
        //std::cout<<"Waiting for X-calibration feedback..."<<std::endl;
        usleep(500000);
    }


    std::cout<<"Now calib Y coordinates..."<<std::endl;
    while(y_current_pass<COORDS_MULTIPASS){
        // 发送Y标定指令到采集端
        std::cout<<"Ready to send a Y calibration instruction..."<<std::endl;
        char wbuf[1]={CALIB_Y};
        size_t n=write(fd, wbuf, 1);
        if(n!=1){
            std::cout<<"Write ttyUSB error, retrying..."<<std::endl;
            continue; // 发送失败，尝试重发
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

    std::cout<<"X & Y coordinates are stable, now to calibrate the projector..."<<std::endl;
    // 输出坐标值
    for(int i=0;i<SENSOR_NUMBER;i++){
        std::cout<<"("<<x_coords[0][i]<<","<<y_coords[0][i]<<")"<<std::endl;
    }

    // 输出透视图像
    output_perspective_image();

    // 等待接收线程结束
    pthread_join(get_result_thread,&thread_ret);

}
