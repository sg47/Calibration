LOCAL_PATH := $(call my-dir)

OPENCV_CAMERA_MODULES:=on
OPENCV_INSTALL_MODULES:=on

include $(CLEAR_VARS)

OPENCV_CAMERA_MODULES:=off

OPENCV_MK_PATH:=/home/fede/Research/inervis_sfm/OpenCV-2.4.6-android-sdk/sdk/native/jni/OpenCV.mk

include $(OPENCV_MK_PATH)


LOCAL_MODULE    := mixed_sample
LOCAL_SRC_FILES := calibration_wrap.cpp MutualCalibration.cpp Chessboard.cpp CataCameraParameters.cpp Cas1DVanishingPoint.cpp RansacVanishingPoint.cpp
LOCAL_CFLAGS    := -frtti
LOCAL_LDLIBS +=  -llog -ldl
include $(BUILD_SHARED_LIBRARY)
