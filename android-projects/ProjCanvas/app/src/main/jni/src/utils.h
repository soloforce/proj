#include <opencv2/opencv.hpp>
#include <vector>

/** Get the centroid of points.
*   @param vector of points
*   @return the centroid of points
*/
cv::Point2d getCentroid(std::vector<cv::Point>& v)
{
    cv::Point2d pt(0,0);
    int n=v.size();

    for(int i=0; i<n; i++){
        pt.x += v[i].x;
        pt.y += v[i].y;
    }

    if(n){
        pt.x /= (double)n;
        pt.y /= (double)n;
    }

    return pt;
}


/** Get the longest contour.
*
*/
int getLargestContour(std::vector< std::vector<cv::Point> >& contours)
{
    int len;
    int pos=0;

    if(contours.size()){
        len=contours[0].size();
        pos=0;
    }

    for(int i=1; i<contours.size(); i++){
        if(contours[i].size()>len){
            len=contours[i].size();
            pos=i;
        }
    }

    return pos;
}

/** Convert a point to a homogeneous coordinate.
*
*/
void point2mat(cv::Point2d pt, cv::Mat& hm)
{
    hm.setTo(cv::Scalar(1.0));
    hm.at<double>(0,0)=pt.x;
    hm.at<double>(1,0)=pt.y;
}

/** Convert a matrix to a point with proper coordinate.
*
*/
cv::Point2d mat2point(cv::Mat m)
{
    cv::Point2d pt(0,0);
    double d=m.at<double>(2,0);
    if(d != 0.0 ){
        pt.x=m.at<double>(0,0)/d;
        pt.y=m.at<double>(1,0)/d;
    }
    return pt;
}

/** sort the vectors by descending size.
*
*/
bool vector_size_comp(std::vector<cv::Point> v1, std::vector<cv::Point> v2)
{
    return (v1.size() > v2.size() );
}
