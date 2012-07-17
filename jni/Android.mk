LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

OPENCV_CAMERA_MODULES:=off

OPENCV_MK_PATH:=/home/li/Workspace/temp/OpenCV-2.4.1-android-bin2/OpenCV-2.4.1/share/opencv/OpenCV.mk

include $(OPENCV_MK_PATH)


LOCAL_MODULE    := mixed_sample
LOCAL_SRC_FILES := calibration_wrap.cpp MutualCalibration.cpp Chessboard.cpp CataCameraParameters.cpp Cas1DVanishingPoint.cpp RansacVanishingPoint.cpp
LOCAL_CFLAGS    := -frtti
LOCAL_LDLIBS +=  -llog -ldl
include $(BUILD_SHARED_LIBRARY)
