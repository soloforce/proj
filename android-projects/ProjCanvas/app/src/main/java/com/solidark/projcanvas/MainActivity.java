package com.solidark.projcanvas;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.drawable.BitmapDrawable;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ImageView;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.Highgui;
import org.opencv.imgproc.Imgproc;


public class MainActivity extends Activity implements CvCameraViewListener2 {
    private static final String TAG = "MainActivity";
    private CameraBridgeViewBase cvCameraView;
    private Mat canvas=null;

    static {
        if(!OpenCVLoader.initDebug()){
            // Error handler here.
        }
        else {
            //System.loadLibrary("opencv_java");
            System.loadLibrary("projcanvas");
        }
    }

    private native void ocvFrame(long addrRGBA, long addrResult);
    private native void ocvRecalibrate(boolean flag);

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_main);
        cvCameraView = (CameraBridgeViewBase) findViewById(R.id.camera_view);

        cvCameraView.enableView();
        cvCameraView.setVisibility(View.VISIBLE);
        cvCameraView.setCvCameraViewListener(this);

    }


    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_calib) {
            //cvCameraView.disableView();
            ocvRecalibrate(true);
            //cvCameraView.enableView();
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    @Override
    public void onCameraViewStarted(int width, int height) {

    }

    @Override
    public void onPause()
    {
        super.onPause();
    }
    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public void onDestroy() {
        super.onDestroy();
    }

    @Override
    public void onResume()
    {
        super.onResume();
        if(cvCameraView!=null) cvCameraView.enableView();
    }

    @Override
    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        if(canvas==null){
            canvas=new Mat(inputFrame.rgba().size(), CvType.CV_8UC3);
        }

        ocvFrame(inputFrame.rgba().getNativeObjAddr(), canvas.getNativeObjAddr());
        return canvas;
    }
}
