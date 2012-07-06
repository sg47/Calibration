package cvg.sfmPipeline.calibration;


import android.app.AlertDialog;
import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.Window;
import android.widget.Button;

public class CalibrationActivity extends Activity{
    private static final String TAG = "Calibration::Activity";

    private MenuItem UseOpencv;
    private MenuItem UseSubpix;
    private CameraView mView;    
    //private CalibView mCalibView;
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
		mView.grabAndProcess();
	}
	
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.i(TAG, "onCreate");
        setContentView(R.layout.main);
        mView = (CameraView)findViewById(R.id.cameraViewer);
        //mCalibView = (CalibView)findViewById(R.id.calibViewer);
    }
    
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

}
