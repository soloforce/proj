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


#define CALIB_X 'X'
#define CALIB_Y 'Y'
#define EXIT_CALIB 'Q'

// 格雷码的bit数，也就是格雷码结构图像的帧数
#define GC_FRAMES 8

// 坐标的采集遍数，只有满足所有遍数采集到的坐标值都相等了，才认为是稳定的坐标，否则必须重新采集
#define COORDS_MULTIPASS_NUMBER 2



unsigned long coords_x[SENSOR_NUMBER];
unsigned long coords_y[SENSOR_NUMBER];

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

void calib(char* buf)
{
    char* xy=strtok(buf,":");
    char* coords=strtok(NULL,":");
    char *s[SENSOR_NUMBER];

    s[0]=strtok(coords,",");
    int i;
    for(i=1;i<SENSOR_NUMBER;i++) s[i]=strtok(NULL,",");

    if(xy[0]==CALIB_X){
        for(i=0;i<SENSOR_NUMBER;i++){
            coords_x[i]=gray2bin(strtoul(s[i],NULL,16));
            printf("%lu\n",coords_x[i]);
        }
    }else if(xy[0]==CALIB_Y){
        for(i=0;i<SENSOR_NUMBER;i++){
            coords_y[i]=gray2bin(strtoul(s[i],NULL,16));
            printf("%lu\n",coords_y[i]);
        }
    }
}

void* get_result( void *ptr )
{
    char buf[1024];
    int fd=*((int*)ptr);
    char* pos;
    int n;

    pos=&buf[0];
    while(1){
        if((n=read(fd, pos, 1))==1 ){
            if(*pos=='\n'){
                *pos=0;



                if(pos!=buf){
                    printf("%s\n",buf);
                    fflush(stdout);
                    calib(buf);
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

    char buf[8]={0};

    while(buf[0]!=EXIT_CALIB){
        memset(buf,0,sizeof buf);
        printf("CMD: ");
        fgets(buf, sizeof buf, stdin);
        int len=strlen(buf);
        if(len>1 && (buf[0]==CALIB_X || buf[0]==CALIB_Y)){

            size_t n=write(fd, buf, 1);
            if(n!=1){
                printf("\n\tWrite ttyUSB error!\n");
                continue;
            }

            //usleep(80000);
            if(buf[0]==CALIB_X){
                for(int i=0; i<GC_FRAMES; i++){
                    cv::imshow(window_name, gcImgH[i]);
                    cv::waitKey(FRAME_INTERVAL);
                }
            }

            if(buf[0]==CALIB_Y){
                for(int i=0; i<GC_FRAMES; i++){
                    cv::imshow(window_name, gcImgV[i]);
                    cv::waitKey(FRAME_INTERVAL);
                }
            }

        }
        usleep(100000);
    }

   // if('c'==cv::waitKey(0)){

        /*
        for(int i=0; i<SYNC_BITS_NUM; i++){
            if(i%2){
                cv::imshow(window_name, white);
                cv::waitKey(FRAME_INTERVAL);
            }else{
                cv::imshow(window_name, black);
                cv::waitKey(FRAME_INTERVAL);
            }
        }*/

        //usleep(500000);
/*
    while(true){
        for(int i=0; i<GC_FRAMES; i++){
            cv::imshow(window_name, gcImgH[i]);
            cv::waitKey(FRAME_INTERVAL);
        }


        for(int i=0; i<GC_FRAMES; i++){
            cv::imshow(window_name, gcImgV[i]);
            cv::waitKey(FRAME_INTERVAL);
        }
   // }

    }*/

    pthread_join(get_result_thread,&thread_ret);

}
