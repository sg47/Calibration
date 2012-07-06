package cvg.sfmPipeline.calibration;

import java.io.IOException;
import java.util.List;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.ImageFormat;
import android.hardware.Camera;
import android.hardware.Camera.PreviewCallback;
import android.os.Handler;
import android.util.AttributeSet;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.ViewGroup;
import android.view.ViewGroup.LayoutParams;
import android.widget.ImageView;

public abstract class AbstractCameraView extends SurfaceView implements SurfaceHolder.Callback {
    private static final String TAG = "Calibration::AbstractSurfaceView";
    
    public static final int CALIB_IMAGE = 0;
    public static final int CAM_PREVIEW = 1;

    private Camera              mCamera;
    private SurfaceHolder       mHolder;
    private int                 mFrameWidth;
    private int                 mFrameHeight;
    private byte[]              mFrame;
    private Bitmap 				mBitmap;
    private ImageView			view;
	private int 				mViewerState = CAM_PREVIEW;
    
    public AbstractCameraView(Context context, AttributeSet attrs) {
        super(context,attrs);
        mHolder = getHolder();
        mHolder.addCallback(this);
        mHolder.setSizeFromLayout();
        
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    public int getFrameWidth() {
        return mFrameWidth;
    }

    public int getFrameHeight() {
        return mFrameHeight;
    }

    public void setPreview() throws IOException {
    	mCamera.setPreviewDisplay(mHolder);
	}

    public boolean openCamera() {
        Log.i(TAG, "openCamera");
        releaseCamera();
        mCamera = Camera.open();
        if(mCamera == null) {
        	Log.e(TAG, "Can't open camera!");
        	return false;
        }
        return true;
    }
    
    public void grabAndProcess(){
    	mCamera.setOneShotPreviewCallback(mPreviewListener);
    }
    
    private PreviewCallback mPreviewListener = new PreviewCallback() {
        public void onPreviewFrame(byte[] data, Camera camera) {
        	mFrame = data;
        	changeSurfaceSize(R.id.cameraViewer, 0, 0);
        	mBitmap = processFrame(mFrame);
        	if (mBitmap != null) {
	    		view.setImageBitmap(mBitmap);
	    		view.setVisibility(View.VISIBLE);
            }        	
        	view.postDelayed(new Runnable(){
				@Override
				public void run() {
		        	view.setVisibility(View.GONE);
		        	changeSurfaceSize(R.id.cameraViewer, 640, 480);
				}}, 500);
        }
    };
       
    public void releaseCamera() {
        Log.i(TAG, "releaseCamera");
        synchronized (this) {
	        if (mCamera != null) {
                mCamera.stopPreview();
                mCamera.setPreviewCallback(null);
                mCamera.release();
                mCamera = null;
            }
        }
        onPreviewStopped();
    }
    
    
    
    public void setupCamera() {
        Log.i(TAG, "setupCamera");
            if (mCamera != null) {
                Camera.Parameters params = mCamera.getParameters();
                // hardcoded ( TODO )
                mFrameWidth = 640;
                mFrameHeight = 480;
                
                params.setPreviewSize(getFrameWidth(), getFrameHeight());
                
                List<String> FocusModes = params.getSupportedFocusModes();
                if (FocusModes.contains(Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO)){
                	params.setFocusMode(Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO);
                }            
                
                mCamera.setParameters(params);
                params = mCamera.getParameters();
                /* Now allocate the buffer */
                int size = params.getPreviewSize().width * params.getPreviewSize().height;
                size  = size * ImageFormat.getBitsPerPixel(params.getPreviewFormat()) / 8;
                mFrame = new byte [size];
                
                Log.i(TAG, "Camera Preview Size: " + params.getPreviewSize().width + "x" + params.getPreviewSize().height);
                
    			try {
    				setPreview();
    			} catch (IOException e) {
    				Log.e(TAG, "mCamera.setPreviewDisplay/setPreviewTexture fails: " + e);
    			}
    				
                /* Notify that the preview is about to be started and deliver preview size */
                onPreviewStarted(params.getPreviewSize().width, params.getPreviewSize().height);

                /* Now we can start a preview */
                mCamera.startPreview();
            }
    }
    
    public void surfaceChanged(SurfaceHolder _holder, int format, int width, int height) {
        Log.i(TAG, "surfaceChanged " + width + "x" + height);
        
    }

    public void surfaceCreated(SurfaceHolder holder) {
        Log.i(TAG, "surfaceCreated");
        setupCamera();
        changeSurfaceSize(R.id.cameraViewer, getFrameWidth(), getFrameHeight());
        view = (ImageView)((View)getParent()).findViewById(R.id.calibViewer);
        view.setVisibility(View.INVISIBLE);
       
    }

    public void surfaceDestroyed(SurfaceHolder holder) {
        Log.i(TAG, "surfaceDestroyed");
        releaseCamera();
    }
    


    private SurfaceView changeSurfaceSize(int idSurface, int width, int height){
    	SurfaceView trueview = (SurfaceView)((View)getParent()).findViewById(idSurface);
		ViewGroup.LayoutParams params = trueview.getLayoutParams();
		params.height = height;
		params.width = width;
		trueview.setLayoutParams(params);
		return trueview;
    }
    
    /* The bitmap returned by this method shall be owned by the child and released in onPreviewStopped() */
    protected abstract Bitmap processFrame(byte[] data);

    /**
     * This method is called when the preview process is being started. It is called before the first frame delivered and processFrame is called
     * It is called with the width and height parameters of the preview process. It can be used to prepare the data needed during the frame processing.
     * @param previewWidth - the width of the preview frames that will be delivered via processFrame
     * @param previewHeight - the height of the preview frames that will be delivered via processFrame
     */
    protected abstract void onPreviewStarted(int previewWidtd, int previewHeight);

    /**
     * This method is called when preview is stopped. When this method is called the preview stopped and all the processing of frames already completed.
     * If the Bitmap object returned via processFrame is cached - it is a good time to recycle it.
     * Any other resources used during the preview can be released.
     */
    protected abstract void onPreviewStopped();

    
}
