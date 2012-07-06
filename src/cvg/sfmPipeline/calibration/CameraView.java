package cvg.sfmPipeline.calibration;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import android.content.Context;
import android.graphics.Bitmap;
import android.util.AttributeSet;
import android.util.Log;

public class CameraView extends AbstractCameraView{
	private static final String TAG = "Calibration::CamView";
    
	public static final boolean MODE_USEOPENCV = true;
	public static final boolean MODE_USESUBPIXEL = false;

	
    private Mat mYuv;
    private Mat mRgba;
    private Mat mGraySubmat;
    private Mat mIntermediateMat;

    private boolean mMode = MODE_USEOPENCV;
	private Bitmap mBitmap;

    public CameraView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }
    
    public void setMode(boolean mode){
    	mMode = mode;
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
        mYuv.put(0, 0, data);
        // TODO: skip this? make everything grayscale no?
        Imgproc.cvtColor(mYuv, mRgba, Imgproc.COLOR_YUV420sp2RGB, 4);
        FindFeatures(mGraySubmat.getNativeObjAddr(), mRgba.getNativeObjAddr(), mMode);
        Bitmap bmp = mBitmap;
        try {
            Utils.matToBitmap(mRgba, bmp);
        } catch(Exception e) {
            Log.e(TAG, "Utils.matToBitmap() throws an exception: " + e.getMessage());
            bmp.recycle();
            bmp = null;
        }

        return bmp;
    }

    public native void FindFeatures(long matAddrGr, long matAddrRgba, boolean mode);

    static {
    	System.loadLibrary("opencv_java");
        System.loadLibrary("mixed_sample");
    }

}
