package cvg.sfmPipeline.calibration;


import java.io.BufferedWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import android.app.AlertDialog;
import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

public class CalibrationActivity extends Activity{
    private static final String TAG = "Calibration::Activity";
    private static TextView mTextNoImgs;
    private static Button mButtonGrab;
    private static TextView mTextRoll;
    private static TextView mTextPitch;
    private static TextView mTextYaw;
    
    public static final int IMAGES_TXT = 0;
    public static final int SENSOR_TXT = 1;
    
    public static float[] latestSensor;
    public static float[] beforeSensor;
    
    private MenuItem UseOpencv;
    @SuppressWarnings("unused")
	private MenuItem UseSubpix;
    private CameraView mView;    
    private SensorManager  mSensorManager;
    private List<Sensor> sensors;

    private List<Integer> logList;
    
    @Override
	protected void onPause() {
        Log.i(TAG, "onPause");
		super.onPause();
		mView.releaseCamera();
	}

	@Override
	protected void onResume() {
        Log.i(TAG, "onResume");
		super.onResume();
		if( !mView.openCamera() ) {
			AlertDialog ad = new AlertDialog.Builder(this).create();  
			ad.setMessage("Fatal error: can't open camera!");   
			ad.show();
		}
	}
	
	public void clickGrabImage(View view){
		setEnabledUI(false);
		beforeSensor = latestSensor;
		mView.grabAndProcess();
	}
	
	public void clickCalibrate(View view){
		mView.calibrationObject.calibrateCamera();
		mView.calibrationObject.mutualCalibrate();
		
	}
	
	public static void updateUI(int what, String noIms, SensorEvent event){
		switch (what){
			case CalibrationActivity.IMAGES_TXT:
				mTextNoImgs.setText("Images: " + noIms);
				break;
			case CalibrationActivity.SENSOR_TXT:
				String valuesStr = "";
	            for (int i = 0; i < event.values.length; i++){ 
	                valuesStr = valuesStr + event.values[i] + ",";
	            }
	            mTextRoll.setText( numberDisplayFormatter(event.values[0]) );
	            mTextPitch.setText( numberDisplayFormatter(event.values[1]) );
	            mTextYaw.setText( numberDisplayFormatter(event.values[2]) );
//	            float norm =  event.values[0]*event.values[0] + event.values[1]*event.values[1] + event.values[2]*event.values[2];
//	            Log.i(TAG, "norm: " + norm);
	            break;
			default:
				break;
		}
		
	}
	
	public static void setEnabledUI(boolean enabled){
		mButtonGrab.setEnabled(enabled);
	}
		
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.i(TAG, "onCreate");
        setContentView(R.layout.main);
        mView = (CameraView)findViewById(R.id.cameraViewer);
        mTextNoImgs = (TextView)findViewById(R.id.textNoImages);
        mButtonGrab = (Button)findViewById(R.id.grabImage);
        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        sensors = mSensorManager.getSensorList(Sensor.TYPE_ALL);
        mTextRoll = (TextView)findViewById(R.id.roll);
        mTextPitch = (TextView)findViewById(R.id.pitch);
        mTextYaw = (TextView)findViewById(R.id.yaw);
        doSensorList();
        for (Sensor sensor : sensors) {
			if(!logList.contains(sensor.getType()) || sensor.getVendor().equalsIgnoreCase("Google Inc."))
				continue;
			mSensorManager.registerListener(mSensorEventListener, sensor, SensorManager.SENSOR_DELAY_FASTEST);
		}
    }
  //--   Sensor event handlers
    private SensorEventListener mSensorEventListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent event) {
        	latestSensor = event.values;
            updateUI(CalibrationActivity.SENSOR_TXT, null, event);
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
        }
    };
    public boolean onCreateOptionsMenu(Menu menu) {
        Log.i(TAG, "onCreateOptionsMenu");
        UseOpencv = menu.add("Normal Accuracy");
        UseSubpix = menu.add("Subpixel Accuracy");
        return true;
    }

    public boolean onOptionsItemSelected(MenuItem item) {
        Log.i(TAG, "Menu Item selected " + item);
        if (item == UseOpencv) {
        	mView.setMode(CameraView.MODE_USEOPENCV);
        } else {
        	mView.setMode(CameraView.MODE_USESUBPIXEL);
        } 
        return true;
    }
    
    private void doSensorList(){
    	logList = new ArrayList<Integer>();
//	    logList.add(Sensor.TYPE_ACCELEROMETER);
//	    logList.add(Sensor.TYPE_GYROSCOPE);
//	    logList.add(Sensor.TYPE_MAGNETIC_FIELD);
//	    logList.add(Sensor.TYPE_LINEAR_ACCELERATION);
//	    logList.add(Sensor.TYPE_ORIENTATION);
//	    logList.add(Sensor.TYPE_GRAVITY);
//	    logList.add(Sensor.TYPE_PRESSURE);
    	logList.add(Sensor.TYPE_ROTATION_VECTOR);
    }
    
    private static String numberDisplayFormatter(float value) {
        String displayedText = Float.toString(value);
        if (value >= 0) {
            displayedText = " " + displayedText;
        }
        if (displayedText.length() > 8) {
            displayedText = displayedText.substring(0, 8);
        }
        while (displayedText.length() < 8) {
            displayedText = displayedText + " ";
        }
        return displayedText;
    }

}
