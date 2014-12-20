LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)


OPENCV_CAMERA_MODULES:=on
OPENCV_INSTALL_MODULES:=on
OPENCV_LIB_TYPE := SHARED

include  /home/genleung/develop/android/ocv4android/sdk/native/jni/OpenCV.mk


LOCAL_SRC_FILES := src/main.cpp \
                   src/processor.cpp \
                   src/bzinterpolating.cpp \
                   src/bzpoint2f.cpp \
                   src/canvas.cpp

LOCAL_LDLIBS += -llog -lm
LOCAL_MODULE := projcanvas
LOCAL_CPPFLAGS += -frtti

include $(BUILD_SHARED_LIBRARY)