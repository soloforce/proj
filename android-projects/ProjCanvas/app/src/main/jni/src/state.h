#ifndef _STATE_H_
#define _STATE_H_

#include <opencv2/opencv.hpp>

class State{
public:
    typedef enum{
        PEN_NA=0,
        PEN_DOWN,
        PEN_UP,
        PEN_MOVE
    }pen_state_t;
public:
    State(){
        penState=PEN_NA;
        ptTouched=ptPrevMapped=ptMapped=cv::Point2d(-1,-1);
    }

    void change(bool touchDetected){
        if(touchDetected){
            if(penState==PEN_DOWN)
                penState=PEN_MOVE;
            else if( (penState==PEN_NA) || (penState==PEN_UP) )
                penState=PEN_DOWN;
        }else{
            if(penState==PEN_UP)
                penState=PEN_NA;
            else if( (penState==PEN_DOWN) || (penState==PEN_MOVE) )
                penState=PEN_UP;
        }
    }

    void change(pen_state_t from, pen_state_t to){
        if(penState==from) penState=to;
    }

    void reset(){
        penState=PEN_NA;
        ptTouched=cv::Point2d(-1,-1);
        ptPrevMapped=cv::Point2d(-1,-1);
        ptMapped=cv::Point2d(-1,-1);
    }

public:
    pen_state_t penState;
    cv::Point2d ptTouched;
    cv::Point2d ptPrevMapped;
    cv::Point2d ptMapped;
};


#endif