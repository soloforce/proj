#include <opencv2/opencv.hpp>

#include <iostream>
#include <bitset>
#include <cmath>
#include <cstdio>
#include <string>


#define PROJ_WIDTH 1024
#define PROJ_HEIGHT 768
#define DESKTOP_WIDTH 2560

#define DX (PROJ_WIDTH/256)
#define DY (PROJ_HEIGHT/256)


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

int main()
{
    std::vector<cv::Point2d> src_points;
    std::vector<cv::Point2d> dst_points, dst_points2;

    src_points.push_back(cv::Point2d(0,0));
    src_points.push_back(cv::Point2d(1023,0));
    src_points.push_back(cv::Point2d(1023,767));
    src_points.push_back(cv::Point2d(0,767));
/*
    src_points.push_back(cv::Point2d(255,191));
    src_points.push_back(cv::Point2d(767,191));
    src_points.push_back(cv::Point2d(767,575));
    src_points.push_back(cv::Point2d(255,575));
*/



    dst_points.push_back(cv::Point2d(200,400));
    dst_points.push_back(cv::Point2d(500,400));
    dst_points.push_back(cv::Point2d(400,600));
    dst_points.push_back(cv::Point2d(100,600));


/*
    dst_points.push_back(cv::Point2d(166*DX,104*DY));
    dst_points.push_back(cv::Point2d(232*DX,106*DY));
    dst_points.push_back(cv::Point2d(234*DX,203*DY));
    dst_points.push_back(cv::Point2d(167*DX,202*DY));
*/

    cv::Rect boundingBox=getBoundingRect(dst_points);
    dst_points2=centerPoints(dst_points, cv::Point2d(boundingBox.x+boundingBox.width/2, boundingBox.y+boundingBox.height/2), cv::Point2d(1024/2, 768/2));
    std::cout<<"dst_points:\n"<<dst_points<<std::endl;
    std::cout<<"dst_points2:\n"<<dst_points2<<std::endl;

    cv::Mat m=pointsToMat(dst_points2);
    std::cout<<m<<std::endl;
    //std::vector<cv::Point2d> vp=matToPoints(m);
    //std::cout<<vp<<"\nXXXXXXXX"<<std::endl;


    //cv::Mat per=getPerspectiveTransform(src_points, dst_points);
    cv::Mat per=findHomography(src_points, dst_points);
    std::cout<<"per:\n"<<per<<std::endl;

    //cv::Mat per2=findHomography(src_points, dst_points2);
    //std::cout<<"per2:\n"<<per2<<std::endl;

    cv::Mat reproj=per.inv()*m;
    std::vector<cv::Point2d> rp_points=matToPoints(reproj);
    std::cout<<reproj<<std::endl;
    std::cout<<rp_points<<std::endl;
    std::cout<<"YYYYYYYY"<<std::endl;
    cv::Rect bbox=getBoundingRect(rp_points);
    std::cout<<bbox<<std::endl;


    float  scale=1.0;
    std::vector<cv::Point2d> rp_points2, prev_points, final_points;

    do{
        rp_points2=rectToPoints(scaleRect(bbox,scale));
        cv::Mat s2=pointsToMat(rp_points2);
        cv::Mat d2=per*s2;
        prev_points=final_points;
        final_points=matToPoints(d2);
        scale+=0.1;
    }while(isPointsInRect(final_points, cv::Rect(0,0,1024,768)));

    final_points=prev_points;


    /*
    cv::Mat trans;
    per.copyTo(trans);
    std::cout<<trans<<std::endl;
    */


    cv::Mat finalH=findHomography(src_points, final_points);
    std::cout<<"finalH\n"<<finalH<<std::endl;

    cv::Mat srcImg=cv::imread("src.png");
    cv::Mat warpImg;
    cv::warpPerspective(srcImg, warpImg, finalH, srcImg.size());

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
