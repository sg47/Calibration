package cvg.sfmPipeline.calibration;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import android.content.Context;
import android.graphics.Bitmap;
import android.util.AttributeSet;
import android.util.Log;

public class CameraView extends AbstractCameraView {
	private static final String TAG = "Calibration::CamView";
    
	//--   calibration object constants
	// TODO: fix capitaliztion:
	public boolean MODE_USEOPENCVCORNER = true;
	public boolean MODE_USERANSCA = false;
	public boolean MODE_OPENCVCALIB = true;
	public boolean MODE_USEONLYIMU  = true;
	public boolean MODE_CHESSBHORZ = true;
	public int checkerRows = 6;
	public int checkerCols = 9;
	
	
    private Mat mYuv;
    private Mat mRgba;
    private Mat mGraySubmat;

	private Bitmap mBitmap;
	
	// the C++ object
	public MutualCalibration calibrationObject;
	
    public CameraView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }
    
    
   
	@Override
	protected void onPreviewStarted(int previewWidtd, int previewHeight) {
		
        // initialize Mats before usage
        mYuv = new Mat(getFrameHeight() + getFrameHeight() / 2, getFrameWidth(), CvType.CV_8UC1);
        // Y channel is gray
        mGraySubmat = mYuv.submat(0, getFrameHeight(), 0, getFrameWidth());
        mRgba = new Mat();        
        mBitmap = Bitmap.createBitmap(previewWidtd, previewHeight, Bitmap.Config.ARGB_8888);
	}

	@Override
	public void grabAndProcess(){
		// initialize calibration object
		if(calibrationObject == null)
		calibrationObject = new MutualCalibration(getFrameHeight(), getFrameWidth(), checkerRows, 
				checkerCols, MODE_USEOPENCVCORNER, MODE_USEONLYIMU, MODE_USERANSCA);
		super.grabAndProcess();
	}
	
	@Override
	protected void onPreviewStopped() {
		
		if (mBitmap != null) {
			mBitmap.recycle();
			mBitmap = null;
		}
		
        // Explicitly deallocate Mats
        if (mYuv != null)
            mYuv.release();
        if (mRgba != null)
            mRgba.release();
        if (mGraySubmat != null)
            mGraySubmat.release();

        mYuv = null;
        mRgba = null;
        mGraySubmat = null;		
	}
	
    @Override
    protected Bitmap processFrame(byte[] data) {
    	float[] afterSensor = CalibrationActivity.latestSensor;
    	float[] sensorValue = interpSensor(CalibrationActivity.beforeSensor, afterSensor);
        mYuv.put(0, 0, data);
        Imgproc.cvtColor(mYuv, mRgba, Imgproc.COLOR_YUV420sp2RGB, 4);
        boolean success;
        
        // Native code call
        if (CalibrationActivity.globalMode == CalibrationActivity.MODE_CHECKERBOARD)
        	success = calibrationObject.tryAddingChessboardImage(mGraySubmat.getNativeObjAddr(), mRgba.getNativeObjAddr());
        else{
        	success = calibrationObject.tryAddingVanishingPointImage(mGraySubmat.getNativeObjAddr(), mRgba.getNativeObjAddr());
        	
        	//also save the before and after images
//        	Bitmap before = Bitmap.createBitmap(getFrameWidth(), getFrameHeight(), Bitmap.Config.ARGB_8888);
//        	Bitmap after = Bitmap.createBitmap(getFrameWidth(), getFrameHeight(), Bitmap.Config.ARGB_8888);
//        	Utils.matToBitmap(mGraySubmat, before);
//        	Utils.matToBitmap(mRgba, after);
//        	try {
//				FileOutputStream input = new FileOutputStream( new File(getDataFolder() + String.format("/%02d_input.png", calibrationObject.getNumberOfImages())) );
//				FileOutputStream output = new FileOutputStream( new File(getDataFolder() + String.format("/%02d_output.png", calibrationObject.getNumberOfImages())) );
//				before.compress(Bitmap.CompressFormat.PNG, 100, input);
//				after.compress(Bitmap.CompressFormat.PNG, 100, output);
//			} catch (FileNotFoundException e) {
//				e.printStackTrace();
//			}
        }
        if (success){
        	if(MODE_USEONLYIMU)
        		calibrationObject.addIMUGravityVector((double)sensorValue[0], (double)sensorValue[1], (double)sensorValue[2]);
        	else
        		calibrationObject.addFullIMURotationByQuaternion((double)sensorValue[0], (double)sensorValue[1], (double)sensorValue[2]);
        }
        Bitmap bmp = mBitmap;
        try {
            Utils.matToBitmap(mRgba, bmp);
        } catch(Exception e) {
            Log.e(TAG, "Utils.matToBitmap() throws an exception: " + e.getMessage());
            bmp.recycle();
            bmp = null;
        }
        CalibrationActivity.updateUI(CalibrationActivity.IMAGES_TXT, calibrationObject.getNumberOfImages(), null);
        return bmp;
    }
    
    @Override
    protected float[] saveSensorData(){
    	float[] afterSensor = CalibrationActivity.latestSensor;
    	return interpSensor(CalibrationActivity.beforeSensor, afterSensor);
    	
    }

    private float[] interpSensor(float[] before, float[] after) {
		float[] retVal = {0,0,0};
		for (int j = 0; j < 3; j++) {
			retVal[j] = 0.5f*before[j] + 0.5f*after[j];
		}
		
		return retVal;
	}

	public native void FindFeatures(long matAddrGr, long matAddrRgba, boolean mode);

    static {
    	System.loadLibrary("opencv_java");
        System.loadLibrary("mixed_sample");
    }

}
