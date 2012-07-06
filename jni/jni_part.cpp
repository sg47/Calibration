
#include <jni.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <android/log.h>

#define LOG_TAG "Calibration::Native::Main"
#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__))

#include "Chessboard.h"

using namespace std;
using namespace cv;


bool detectCorners(Mat * inputImage, const Size & boardSize, Mat * outputImage, bool mode)
{
	vcharge::Chessboard chessboard(boardSize, *inputImage);
	chessboard.findCorners(mode);// true then runs simple OpenCV checkerboard pattern corner detection
	chessboard.getSketch().copyTo(*outputImage);
	return chessboard.cornersFound();
}
extern "C" {
JNIEXPORT void JNICALL Java_cvg_sfmPipeline_calibration_CameraView_FindFeatures
(JNIEnv* env, jobject thiz, jlong addrGray, jlong addrRgba, jboolean mode)
{
    Mat* pMatGr=(Mat*)addrGray;
    Mat* pMatRgb=(Mat*)addrRgba;
    Size boardSize(6, 9);
    bool crnrsFound = detectCorners(pMatGr, boardSize, pMatRgb, (bool)mode);
    LOGI("Detect corners %s",(crnrsFound)?"successful":"failed");
}
}
