package sq.rogue.rosettadrone.autolanding;

import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_NONE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2GRAY;
import static org.opencv.imgproc.Imgproc.FILLED;
import static org.opencv.imgproc.Imgproc.MORPH_CLOSE;
import static org.opencv.imgproc.Imgproc.MORPH_RECT;
import static org.opencv.imgproc.Imgproc.RETR_LIST;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY;
import static org.opencv.imgproc.Imgproc.circle;
import static org.opencv.imgproc.Imgproc.connectedComponentsWithStats;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.getStructuringElement;
import static org.opencv.imgproc.Imgproc.moments;
import static org.opencv.imgproc.Imgproc.morphologyEx;
import static org.opencv.imgproc.Imgproc.threshold;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.graphics.PointF;
import android.graphics.SurfaceTexture;
import android.media.MediaFormat;
import android.os.Bundle;
import android.util.Log;
import android.view.TextureView;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Moments;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;
import java.util.Timer;
import java.util.TimerTask;

import dji.common.error.DJIError;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.product.Model;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.products.Aircraft;
import sq.rogue.rosettadrone.R;
import sq.rogue.rosettadrone.RDApplication;
import sq.rogue.rosettadrone.settings.Tools;

public class TestingActivity extends Activity implements TextureView.SurfaceTextureListener, View.OnClickListener {

    private static final String TAG = "TestingActivity";

    private Button takeoffBtn;
    private Button flightControlBtn;
    private Button testBtn;
    private Button targetBtn;
    private Button gimbalBtn;
    private Button precisionLandingBtn;
    private ImageView imageView;
    protected TextureView videoSurface = null;

    protected VideoFeeder.VideoDataListener videoDataListener = null;
    protected VideoFeeder.VideoFeed videoFeed = null;
    protected DJICodecManager codecManager = null;

    private FlightController flightController;
    private FlightControlData flightControlData;
    private SendFlightControlDataTask sendFlightControlDataTask;
    private Timer sendFlightControlData;

    private BaseLoaderCallback loaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.d(TAG, "OpenCV loaded successfully");
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
                break;
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_testing);
        initFlightControl();
        initUI();

        Camera camera = RDApplication.getProductInstance().getCamera();

        //video read in 1
        videoFeed = VideoFeeder.getInstance().provideTranscodedVideoFeed();
        videoFeed.addVideoDataListener(new VideoFeeder.VideoDataListener() {
            @Override
            public void onReceive(byte[] videoBuffer, int size) {
                Log.d(TAG, "Video size:" + size);
            }
        });
    }

    @Override
    public void onResume() {
        super.onResume();

//        OpenCVLoader.initAsync()
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, loaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
        BaseProduct product = RDApplication.getProductInstance();


        if(product == null || !product.isConnected()) {

        }else {
            if(videoSurface != null) {
                videoSurface.setSurfaceTextureListener(this);
            }
            if(!product.getModel().equals(Model.UNKNOWN_AIRCRAFT)) {
                VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(videoDataListener);
            }
        }
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        if(sendFlightControlDataTask != null) {
            sendFlightControlDataTask = null;
            sendFlightControlDataTask.cancel();
        }
        if(sendFlightControlData != null) {
            sendFlightControlData.cancel();;
            sendFlightControlData.purge();
            sendFlightControlData = null;
        }
        flightController.setVirtualStickModeEnabled(false, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
//                Tools.showToast(this, "Unable to control");
            }
        });
    }

    private void initFlightControl() {

        if(RDApplication.getProductInstance() != null) {
            flightController = ((Aircraft) RDApplication.getProductInstance()).getFlightController();
            flightController.setVirtualStickModeEnabled(false, new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                }
            });

        }

        flightController.setRollPitchControlMode(RollPitchControlMode.ANGLE);
        flightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
        flightController.setYawControlMode(YawControlMode.ANGLE);
        flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);
    }

    private void initUI() {
        takeoffBtn = findViewById(R.id.takeoffBtn);
        testBtn = findViewById(R.id.testBtn);
        flightControlBtn = findViewById(R.id.fcBtn);
        targetBtn = findViewById(R.id.targetBtn);
        gimbalBtn = findViewById(R.id.gimbalBtn);
        precisionLandingBtn = findViewById(R.id.precisionLandingBtn);

        takeoffBtn.setOnClickListener(this);
        testBtn.setOnClickListener(this);
        flightControlBtn.setOnClickListener(this);
        targetBtn.setOnClickListener(this);
        gimbalBtn.setOnClickListener(this);
        precisionLandingBtn = findViewById(R.id.precisionLandingBtn);

        videoSurface = findViewById(R.id.videoPreviewerSurface);
        if(videoSurface != null) {
            videoSurface.setSurfaceTextureListener(this);
        }
        videoSurface.setVisibility(View.VISIBLE);

        imageView = findViewById(R.id.matshowV);
    }

    public void onClick(View v){
        switch (v.getId()) {
            case R.id.testBtn:{
                testVideoDataReadInAndOpencv();
                break;
            }
            case R.id.fcBtn:{                  //test flight control
                Tools.showToast(this, "BREAK A LEG :D");
                Intent intent = new Intent(this, FlightControlActivity.class);
                startActivity(intent);
                break;
            }
            case R.id.takeoffBtn:{
                ((Aircraft)RDApplication.getProductInstance()).getFlightController().
                        startLanding(new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {
                            }
                        });
                break;
            }
            case R.id.targetBtn:{
                Tools.showToast(this, "Able to show");
                Log.d(TAG, "Able to show log");
            }
            case R.id.gimbalBtn:{
                GimbalRotateTimerTask gimbalRotateTimerTask = new GimbalRotateTimerTask();
                Timer gimbalRotateTimer = new Timer();
                gimbalRotateTimer.schedule(gimbalRotateTimerTask, 1000, 200);
                break;
            }
            case R.id.precisionLandingBtn:{
                PrecisionLandingController precisionLandingController = new PrecisionLandingController();
                precisionLandingController.startPrecisionLanding();
                break;
            }
            default: break;
        }
    }




    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        if(codecManager == null) {
            codecManager = new DJICodecManager(this, surface, width, height);
        }
    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {

    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        if (codecManager != null) {
            codecManager.cleanSurface();
            codecManager = null;
        }
        return false;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {

    }



    //---------------test method-------------
    private class SendFlightControlDataTask extends TimerTask {
        @Override
        public void run() {
            Log.d(TAG, "Sending flight control data……");
            ((Aircraft)RDApplication.getProductInstance())
                    .getFlightController()
                    .sendVirtualStickFlightControlData(new FlightControlData(1,1, 1, 1),
                            new CommonCallbacks.CompletionCallback() {
                                @Override
                                public void onResult(DJIError djiError) {
                                    if(djiError != null){
                                        Log.d(TAG, "Flight control error" + djiError.getDescription());
                                    }
                                }
                            });
        }
    }

    private static class GimbalRotateTimerTask extends TimerTask {
        @Override
        public void run() {

            RDApplication.getProductInstance().getGimbal().
                    rotate(new Rotation.Builder().pitch(3)
                            .mode(RotationMode.RELATIVE_ANGLE)
                            .yaw(Rotation.NO_ROTATION)
                            .roll(Rotation.NO_ROTATION)
                            .time(0)
                            .build(), new CommonCallbacks.CompletionCallback() {

                        @Override
                        public void onResult(DJIError error) {
                            if(error != null) {
                                Log.d(TAG, "Gimbal error" + error.getDescription());
                            }
                        }
                    });

        }
    }

    private void testVideoDataReadInAndOpencv() {
        //this method is create only for testing
        //2022-1-16 it works quite well than I expect :D

//        for(int i=0; i<50; i++) {
            int videoWidth = codecManager.getVideoWidth();
            int videoHeight = codecManager.getVideoHeight();
            byte[] RGBAData = codecManager.getRgbaData(videoWidth, videoHeight);

            if(RGBAData != null) {
                Log.d(TAG, "Get RGBAData successfully, data size:" + RGBAData.length);

                //load mat
                Mat frame = new Mat(videoHeight, videoWidth, CvType.CV_8UC4);
                frame.put(0, 0, RGBAData);
                Log.d(TAG, "FRAME MESSAGE " + "cols: " + frame.cols() + " rows: " + frame.rows());
                //test the detect
                Mat frameDealing = new Mat();
                List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
                Point targetPoint = null;

                //RGB 2 gray
                cvtColor(frame, frameDealing, COLOR_RGB2GRAY);

                //gray 2 binary frame
                threshold(frameDealing, frameDealing, 220 ,250, THRESH_BINARY);   // ? enable threshold value for self-adaptation

                //CLOSE
                Mat element = getStructuringElement(MORPH_RECT, new Size(7, 7));
                morphologyEx(frameDealing, frameDealing, MORPH_CLOSE, element);

                //get contours
                findContours(frameDealing, contours, new Mat(), RETR_TREE, CHAIN_APPROX_NONE);
                connectedComponentsWithStats(frameDealing, new Mat(), new Mat(), new Mat());
                getBiggestContours(contours);
                int max = getBiggestContoursNumber(contours);
                if(max == 0) {
                    Log.d(TAG, "There is no contour detected in the frame.");
                }

                //get center
                Moments Moments = moments(contours.get(max));
                targetPoint = new Point(Moments.m10 / Moments.m00, Moments.m01 / Moments.m00);
                Log.d(TAG, "POINT MESSAGE: " + targetPoint);
                Tools.showToast(this, targetPoint.toString());

                //visualizing the result
                drawContours(frame, contours, max, new Scalar(0, 0, 255) ,FILLED);
                circle(frame, targetPoint,40, new Scalar(0, 255, 0), FILLED);

                //mat 2 bitmap
                Bitmap bitmap = null;
                if(frame.cols() > 0 && frame.rows() > 0) {
                    bitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
                    Utils.matToBitmap(frame, bitmap);
                    int h = 360;
                    int w = h*bitmap.getWidth()/ bitmap.getHeight();
                    Log.d(TAG, "test"+frame.width()+"1"+frame.cols()+"2"+frame.height()+"3"+frame.rows());
                    Log.d(TAG, "w: " + w + " h: " + h);
                    Log.d(TAG, "bitmap: " + " width: " + bitmap.getWidth() + " height: " + bitmap.getWidth());
                } else {
                    Log.e(TAG, "fail to create the bitmap");
                }
                //adjust view to show video data and detection result
                videoSurface.setVisibility(View.GONE);
                imageView.setImageBitmap(bitmap);
                imageView.setVisibility(View.VISIBLE);

            } else {
                Log.e(TAG, "Can't get RGBAData!");
            }
//        }

    }


    private int getBiggestContoursNumber(List<MatOfPoint> contours) {
        double max_area = contourArea(contours.get(0));
        int max = 0;
        for(int i=0; i< contours.size(); i++){
            double area = contourArea((contours.get(i)));
            if(area > max_area){
                max_area = area;
                max = i;
            }
        }
        return max;
    }

    private void getBiggestContours(List<MatOfPoint> contours) {
        Iterator<MatOfPoint> each = contours.iterator();
        MatOfPoint wrapper = contours.get(0);
        double maxArea = contourArea(wrapper);
        while(each.hasNext()){
            wrapper = each.next();
            double area = contourArea(wrapper);
            if(area < maxArea){
                each.remove();
            }
        }
    }
}

