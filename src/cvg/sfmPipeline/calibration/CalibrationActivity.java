package cvg.sfmPipeline.calibration;


import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import android.app.AlertDialog;
import android.app.Activity;
import android.content.Context;
import android.content.DialogInterface;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.KeyEvent;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuItem;
import android.view.SubMenu;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

public class CalibrationActivity extends Activity{
    private static final String TAG = "Calibration::Activity";
    
    //--   App State
    private boolean mIsCalibrated = false;
    private boolean mCalibImagesStarted = false;
    
    //--   UI elements
    private CameraView mView; 
    private static TextView mTextNoImgs;
    private static Button mButtonGrab;
    private static Button mButtonCalib;
    private static TextView mTextRoll;
    private static TextView mTextPitch;
    private static TextView mTextYaw;
    	//- matrix views
    private static TextView mMatLabel;
    private static TextView mMat_1_1;
    private static TextView mMat_1_2;
    private static TextView mMat_1_3;
    private static TextView mMat_2_1;
    private static TextView mMat_2_2;
    private static TextView mMat_2_3;
    private static TextView mMat_3_1;
    private static TextView mMat_3_2;
    private static TextView mMat_3_3;
    
    //--  Update UI mode
    public static final int IMAGES_TXT = 0;
    public static final int SENSOR_TXT = 1;
    
    //--   Menu Items
	private static final int SUB_PIX_ACC = 0;
	private static final int USE_RANSAC = 1;
	private static final int OPENCV_CALIB = 3;
	private static final int ONLY_IMU = 4;
	private static final int CHESS_HORIZ = 5;
	private static final int PRESET_1 = 6;
	private static final int PRESET_2 = 7;
	private static final int USER_DEFINED = 8;
	private int checkH = 6;
	private int checkW = 9;
	
	//--   Sensor readings buffers
    public static float[] latestSensor;
    public static float[] beforeSensor;
    private SensorManager  mSensorManager;
    private List<Sensor> sensors;
    private List<Integer> logList;

	public static String filePathUniqueIdentifier;
    
    @Override
	protected void onPause() {
        Log.i(TAG, "onPause");
		super.onPause();
		mView.releaseCamera();
	}

	@Override
	protected void onResume() {
        Log.i(TAG, "onResume");
        onCreate(null);
		super.onResume();
		if( !mView.openCamera() ) {
			AlertDialog ad = new AlertDialog.Builder(this).create();  
			ad.setMessage("Fatal error: can't open camera!");   
			ad.show();
		}
	}
	
	public void clickGrabImage(View view){
		mCalibImagesStarted = true;
		setEnabledUI(false);
		if(!mIsCalibrated){
			beforeSensor = latestSensor;
			mView.grabAndProcess();
		}else
		{	
			Log.i(TAG,"Now instead, save the image and sensor values in SD");
			mView.grabAndSave();
			mButtonCalib.setEnabled(false);
		}
	}
	
	public void clickCalibrate(View view){
		if(mView.calibrationObject == null){
			(Toast.makeText(this, "Please take 3 images of a calibration pattern first!", Toast.LENGTH_SHORT)).show();
			return;
		}
		if(mView.calibrationObject.getNumberOfImages() < 3){
			(Toast.makeText(this, "Please take 3 images of a calibration pattern first!", Toast.LENGTH_SHORT)).show();
			return;
		}
		
		setEnabledUI(false);
		mView.calibrationObject.calibrateCamera();
		mView.calibrationObject.mutualCalibrate();
		double[] data1 = new double[9];
		mView.calibrationObject.getRotationMatrix(data1);
		displayMatrix("Rot CAM2IMU", data1);
		mIsCalibrated = true;
		setEnabledUI(true);
		mButtonCalib.setEnabled(false);
		mButtonGrab.setText("Grab Image");
		
	}
	
	public static void updateUI(int what, String noIms, SensorEvent event){
		switch (what){
			case CalibrationActivity.IMAGES_TXT:
				mTextNoImgs.setText("Calib Images: " + noIms);
				break;
			case CalibrationActivity.SENSOR_TXT:
				String valuesStr = "";
	            for (int i = 0; i < event.values.length; i++){ 
	                valuesStr = valuesStr + event.values[i] + ",";
	            }
	            mTextRoll.setText( numberDisplayFormatter(event.values[0]) );
	            mTextPitch.setText( numberDisplayFormatter(event.values[1]) );
	            mTextYaw.setText( numberDisplayFormatter(event.values[2]) );
	            break;
			default:
				break;
		}
		
	}
	
	public static void displayMatrix(String label, double[] vals){
		mMatLabel.setText(label);
		mMat_1_1.setText(numberDisplayFormatter(vals[0]));
		mMat_1_2.setText(numberDisplayFormatter(vals[1]));
		mMat_1_3.setText(numberDisplayFormatter(vals[2]));
		mMat_2_1.setText(numberDisplayFormatter(vals[3])); 
		mMat_2_2.setText(numberDisplayFormatter(vals[4]));
		mMat_2_3.setText(numberDisplayFormatter(vals[5]));
		mMat_3_1.setText(numberDisplayFormatter(vals[6]));
		mMat_3_2.setText(numberDisplayFormatter(vals[7]));
		mMat_3_3.setText(numberDisplayFormatter(vals[8]));
	}
	
	public static void setEnabledUI(boolean enabled){
		mButtonGrab.setEnabled(enabled);
		mButtonCalib.setEnabled(enabled);
	}
		
    @Override
    public void onCreate(Bundle savedInstanceState) {
        
    	 Date date = new Date();
         SimpleDateFormat fmt = new SimpleDateFormat("MM-dd_HH-mm-SS");
         filePathUniqueIdentifier = fmt.format(date);
    	
    	super.onCreate(savedInstanceState);
        Log.i(TAG, "onCreate");
        setContentView(R.layout.main);
        mView = (CameraView)findViewById(R.id.cameraViewer);
        mTextNoImgs = (TextView)findViewById(R.id.textNoImages);
        mButtonGrab = (Button)findViewById(R.id.grabImage);
        mButtonCalib = (Button)findViewById(R.id.calibrate);
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
        
        mMatLabel = (TextView) findViewById(R.id.matrixLabel);
        mMat_1_1 = (TextView) findViewById(R.id.matrix_1_1);
        mMat_1_2 = (TextView) findViewById(R.id.matrix_1_2);
        mMat_1_3 = (TextView) findViewById(R.id.matrix_1_3);
        mMat_2_1 = (TextView) findViewById(R.id.matrix_2_1);
        mMat_2_2 = (TextView) findViewById(R.id.matrix_2_2);
        mMat_2_3 = (TextView) findViewById(R.id.matrix_2_3);
        mMat_3_1 = (TextView) findViewById(R.id.matrix_3_1);
        mMat_3_2 = (TextView) findViewById(R.id.matrix_3_2);
        mMat_3_3 = (TextView) findViewById(R.id.matrix_3_3);
       
        
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
        menu.add(0, SUB_PIX_ACC, 0, "Subpixel corners").setCheckable(true).setChecked(false);
        menu.add(0, USE_RANSAC, 0, "RANSAC").setCheckable(true).setChecked(false);
        menu.add(0, OPENCV_CALIB, 0, "OpenCV calibration").setCheckable(true).setChecked(true);
        menu.add(0, ONLY_IMU, 0, "Use only IMU").setCheckable(true).setChecked(true);
//        menu.add(0, CHESS_HORIZ, 0, "Horizontal").setCheckable(true).setChecked(false);
        
        SubMenu sm = menu.addSubMenu("Checkerboard Options");
        sm.add(1, PRESET_1, 0, "Preset 1");
        sm.add(1, PRESET_2, 0, "Preset 2");
        sm.add(1, USER_DEFINED, 0, "User Defined");
        sm.setGroupCheckable(1, true, true);
        return true;
    }
    
    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        Log.i(TAG, "Menu Item selected " + item);
        super.onOptionsItemSelected(item);
        if(mCalibImagesStarted) return true;
        switch(item.getItemId()){
        	case SUB_PIX_ACC:
		        if (item.isChecked()) item.setChecked(false);
				else item.setChecked(true);
		        mView.MODE_USEOPENCVCORNER = item.isChecked();
        		return true;
        	case USE_RANSAC:
		        if (item.isChecked()) item.setChecked(false);
				else item.setChecked(true);
		        mView.MODE_USERANSCA = item.isChecked();
        		return true;
        	case OPENCV_CALIB:
		        if (item.isChecked()) item.setChecked(false);
				else item.setChecked(true);
		        mView.MODE_OPENCVCALIB = item.isChecked();
        		return true;
        	case ONLY_IMU:
		        if (item.isChecked()) item.setChecked(false);
				else item.setChecked(true);
		        mView.MODE_USEONLYIMU = item.isChecked();
        		return true;
        	case CHESS_HORIZ:
		        if (item.isChecked()) item.setChecked(false);
				else item.setChecked(true);
		        mView.MODE_CHESSBHORZ = item.isChecked();
        		return true;
        	case PRESET_1:
        		item.setChecked(true);
        		return true;
        	case PRESET_2:
        		item.setChecked(true);
        		return true;
        	case USER_DEFINED:
				item.setChecked(true);
				changeCheckerboard();
				
        		return true;
        }
		return true; 
        
    }
    
    private void changeCheckerboard() {
    	AlertDialog.Builder alertBuilder = new AlertDialog.Builder(this);
    	View twoEdits = getLayoutInflater().inflate(R.layout.checkersize, null);
    	alertBuilder.setTitle("Change checkerboard size (rowsxcols");
    	alertBuilder.setView(twoEdits);
    	
    	final EditText mEditCols = (EditText)twoEdits.findViewById(R.id.checkerCols);
    	final EditText mEditRows = (EditText)twoEdits.findViewById(R.id.checkerRows);
    	mEditCols.setImeOptions(EditorInfo.IME_FLAG_NO_EXTRACT_UI);
    	mEditRows.setImeOptions(EditorInfo.IME_FLAG_NO_EXTRACT_UI);
    	mEditCols.setText(checkW+"");
    	mEditRows.setText(checkH+"");
    	alertBuilder
    		.setCancelable(true)
    		.setPositiveButton("Change", new DialogInterface.OnClickListener() {
				@Override
				public void onClick(DialogInterface dialog, int which) {
					checkW = Integer.parseInt(mEditCols.getText().toString());
					checkH = Integer.parseInt(mEditRows.getText().toString());
					mView.checkerCols = checkW;
					mView.checkerRows = checkH;
					dialog.dismiss();
				}
			});
    	AlertDialog alert = alertBuilder.create();
    	alert.show();    		
	}

	@Override
    public boolean onPrepareOptionsMenu(Menu menu){
    	super.onPrepareOptionsMenu(menu);
    	if (mCalibImagesStarted){
    		for (int i = 0; i < menu.size(); i++) {
				menu.getItem(i).setEnabled(false);
			}
    	}
    	String str = String.format("User defined" );
    	menu.findItem(USER_DEFINED).setTitle(str);
		return true;
    }
    
    private void doSensorList(){
    	logList = new ArrayList<Integer>();
//	    logList.add(Sensor.TYPE_ACCELEROMETER);
//	    logList.add(Sensor.TYPE_GYROSCOPE);
//	    logList.add(Sensor.TYPE_MAGNETIC_FIELD);
//	    logList.add(Sensor.TYPE_LINEAR_ACCELERATION);
//	    logList.add(Sensor.TYPE_ORIENTATION);
	    logList.add(Sensor.TYPE_GRAVITY);
//	    logList.add(Sensor.TYPE_PRESSURE);
//    	logList.add(Sensor.TYPE_ROTATION_VECTOR);
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
    private static String numberDisplayFormatter(double value) {
        String displayedText = String.format("%.3f", (float)value);
        if (value >= 0) {
            displayedText = " " + displayedText;
        }
        if (displayedText.length() > 5) {
            displayedText = displayedText.substring(0, 5);
        }
        while (displayedText.length() < 5) {
            displayedText = displayedText + " ";
        }
        return displayedText;
    }
    
    @Override
    public boolean onKeyDown(int keyCode, KeyEvent event) {
        if (keyCode == KeyEvent.KEYCODE_BACK) {
        	AlertDialog.Builder alertBuilder = new AlertDialog.Builder(this);
        	alertBuilder.setTitle("Exit application?");
        	alertBuilder
        		.setCancelable(true)
        		.setPositiveButton("Yes", new DialogInterface.OnClickListener() {
					
					@Override
					public void onClick(DialogInterface dialog, int which) {
						finish();
					}
				})
				.setNegativeButton("No", new DialogInterface.OnClickListener() {
					
					@Override
					public void onClick(DialogInterface dialog, int which) {
						dialog.cancel();
					}
				});
        	AlertDialog alert = alertBuilder.create();
        	alert.show();
        	return super.onKeyDown(KeyEvent.KEYCODE_BACK, event);
        }
        if (keyCode == KeyEvent.KEYCODE_MENU){
        	return super.onKeyDown(KeyEvent.KEYCODE_MENU, event);
        }
        
        return super.onKeyDown(KeyEvent.KEYCODE_MENU, event);
    }
    
}
