#include "bzpoint.hpp"


BZPoint2f::~BZPoint2f()
{
    // clean up job goes here
}


BZPoint2f::BZPoint2f()
{
    x=y=-1;
    cpt[0]=cpt[1]=cv::Point2f(-1,-1);
    smoothFactor=DEFAULT_SMOOTH_FACTOR;
    interpolated=false;
}


BZPoint2f::BZPoint2f(int a, int b)
{
    x=a;
    y=b;
    cpt[0]=cpt[1]=cv::Point2f(-1,-1);
    smoothFactor=DEFAULT_SMOOTH_FACTOR;
    interpolated=false;
}


BZPoint2f::BZPoint2f(const BZPoint2f& bpt)
{
    x=bpt.x;
    y=bpt.y;
    cpt[0]=bpt.cpt[0];
    cpt[1]=bpt.cpt[1];
    smoothFactor=bpt.smoothFactor;
    interpolated=false;
}

BZPoint2f::BZPoint2f(const cv::Point2f& pt)
{
    x=pt.x;
    y=pt.y;
    smoothFactor=DEFAULT_SMOOTH_FACTOR;
    interpolated=false;
}


BZPoint2f& BZPoint2f::operator=(const BZPoint2f& bpt)
{
    x=bpt.x;
    y=bpt.y;
    cpt[0]=bpt.cpt[0];
    cpt[1]=bpt.cpt[1];
    smoothFactor=bpt.smoothFactor;
    interpolated=false;
}

bool BZPoint2f::operator == (const BZPoint2f& bpt)
{
    return ( (x==bpt.x) && (y==bpt.y) );
}
bool BZPoint2f::operator == (const cv::Point2f& bpt)
{
    return ( (x==bpt.x) && (y==bpt.y) );
}

BZPoint2f& BZPoint2f::operator=(const cv::Point2f& pt)
{
    x=pt.x;
    y=pt.y;
    smoothFactor=DEFAULT_SMOOTH_FACTOR;
    interpolated=false;
}

float BZPoint2f::getSmoothFactor() const
{
    return smoothFactor;
}

void BZPoint2f::setSmoothFactor(float sm)
{
    if(sm<0.0) sm=0.0;
    if(sm>1.0) sm=1.0;
    smoothFactor=sm;
}

