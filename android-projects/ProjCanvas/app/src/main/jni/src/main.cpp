#include "com_solidark_projcanvas_MainActivity.h"
#include "processor.h"


static Processor ocvProcessor;

// valid region where the ir-point could be observed
const cv::Rect CAMERA_REGION(
	cv::Point(220,2), // left-up point
	cv::Point(1278,718) // right-bottom
);

const cv::Rect CANVAS_REGION(
	cv::Point(220,2),
	cv::Point(1278,718)
);



JNIEXPORT void JNICALL Java_com_solidark_projcanvas_MainActivity_ocvFrame(JNIEnv* env, jobject, jlong addrRGBA, jlong addrResult)
{
    cv::Mat& frame = *(cv::Mat*)addrRGBA;
    cv::Mat& result = *(cv::Mat*)addrResult;

    if(!ocvProcessor.isInitialized()){
        ocvProcessor.init(result, CAMERA_REGION, CANVAS_REGION);
    }else{
        ocvProcessor.process(frame);
    }


}


JNIEXPORT void JNICALL Java_com_solidark_projcanvas_MainActivity_ocvRecalibrate(JNIEnv *, jobject, jboolean flag)
{
    if(flag){
        ocvProcessor.reset();
    }
}