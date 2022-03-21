package sq.rogue.rosettadrone.autolanding;

import static org.opencv.imgproc.Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_NONE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2GRAY;
import static org.opencv.imgproc.Imgproc.COLOR_YUV2BGR;
import static org.opencv.imgproc.Imgproc.COLOR_YUV2BGR_NV12;
import static org.opencv.imgproc.Imgproc.COLOR_YUV2GRAY_420;
import static org.opencv.imgproc.Imgproc.COLOR_YUV2RGB_I420;
import static org.opencv.imgproc.Imgproc.RETR_LIST;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY_INV;
import static org.opencv.imgproc.Imgproc.adaptiveThreshold;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.minAreaRect;
import static org.opencv.imgproc.Imgproc.moments;
import static org.opencv.imgproc.Imgproc.resize;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.PixelFormat;
import android.graphics.PointF;
import android.graphics.Rect;
import android.graphics.SurfaceTexture;
import android.graphics.YuvImage;
import android.media.MediaFormat;
import android.os.Bundle;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;

import org.greenrobot.eventbus.EventBus;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Moments;

import java.io.ByteArrayOutputStream;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.concurrent.TimeUnit;

import dji.common.error.DJIError;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.product.Model;
import dji.common.util.CommonCallbacks;
import dji.lidar_map.my_point_3d.Point3D;
import dji.midware.data.model.P3.Ta;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;
import sq.rogue.rosettadrone.R;
import sq.rogue.rosettadrone.RDApplication;
import sq.rogue.rosettadrone.settings.Tools;

public class TestingActivity extends Activity implements TextureView.SurfaceTextureListener, SurfaceHolder.Callback, View.OnClickListener {

    private static final String TAG = "TestingActivity";

    private Button targetAndGimbalTestBtn;
    private Button targetTestBtn;
    private Button flightTestBtn;
    private Button targetAndFlightTestBtn;
    private Button fullStartTestBtn;
    private Button exitTestBtn;
    private Button multi1TestBtn;
    private Button multi2TestBtn;
    private ImageView imageView;
    protected TextureView videoSurface = null;
    private SurfaceView drawingSurface = null;
    private EditText pidPET;
    private EditText pidIET;
    private EditText pidDET;


    protected VideoFeeder.VideoDataListener videoDataListener = null;
    protected VideoFeeder.VideoFeed videoFeed = null;
    protected DJICodecManager codecManager = null;

    private FlightController flightController;
    private Thread FCTestThread = null;
    private Thread targetDetection = null;
    private TargetDetect targetDetect = null;
    private DrawingLandingPointThread drawingLandingPointThread;
    private Bitmap bitmap = null;
    private Mat frame = null;
    private boolean keep = false;
    public byte[] yuv = null;

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
            //raw H264 video data
            @Override
            public void onReceive(byte[] videoBuffer, int size) {
                Log.d(TAG, "Video size:" + size);
            }
        });
    }

    

    @Override
    public void onResume() {
        super.onResume();

        //load opencv
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
        targetAndGimbalTestBtn = findViewById(R.id.targetAndGimbalTestBtn);
        flightTestBtn = findViewById(R.id.flightTestBtn);
        targetTestBtn = findViewById(R.id.targetTestBtn);
        targetAndFlightTestBtn = findViewById(R.id.targetAndFlightTestBtn);
        fullStartTestBtn = findViewById(R.id.fullStartTestBtn);
        exitTestBtn = findViewById(R.id.exitTestBtn);
        multi1TestBtn = findViewById(R.id.multiTest1Btn);
        multi2TestBtn = findViewById(R.id.multiTest2Btn);
//        longitudeET = findViewById(R.id.longitudeET);
//        latitudeET = findViewById(R.id.latitudeET);
//        altitudeET = findViewById(R.id.altitudeET);
        pidPET = findViewById(R.id.pidPET);
        pidIET = findViewById(R.id.pidIET);
        pidDET = findViewById(R.id.pidDET);

        targetAndGimbalTestBtn.setOnClickListener(this);
        flightTestBtn.setOnClickListener(this);
        targetTestBtn.setOnClickListener(this);
        targetAndFlightTestBtn.setOnClickListener(this);
        fullStartTestBtn.setOnClickListener(this);
        exitTestBtn.setOnClickListener(this);
        multi1TestBtn.setOnClickListener(this);
        multi2TestBtn.setOnClickListener(this);

        videoSurface = findViewById(R.id.videoPreviewerSurface);
        if(videoSurface != null) {
            videoSurface.setSurfaceTextureListener(this);
        }
        videoSurface.setVisibility(View.VISIBLE);

        imageView = findViewById(R.id.matshowV);

        drawingSurface = findViewById(R.id.drawingSurface);
        drawingSurface.getHolder().addCallback(TestingActivity.this);
        drawingSurface.setZOrderOnTop(true);
        drawingSurface.getHolder().setFormat(PixelFormat.TRANSPARENT);
        drawingSurface.setVisibility(View.VISIBLE);
    }

    VisualLanding visualLanding;
    float a=0, b=0, c=0;
    public void onClick(View v){
        switch (v.getId()) {
            case R.id.targetAndGimbalTestBtn:{
                //test for the 'check' before visual landing
                Tools.showToast(this, "Abandoned.");
                yuvTest();
                break;
            }
            case R.id.targetTestBtn:{
                keep = true;
                testTargetDetect();
                break;
            }
            case R.id.flightTestBtn:{
                targetDetect = new TargetDetect(this, codecManager.getVideoWidth(), codecManager.getVideoHeight());
                targetDetection = new Thread(targetDetect);
                targetDetection.start();
                break;
            }
            case R.id.targetAndFlightTestBtn:{
                FCTestThread = new Thread(new VisualLandingFlightControl(true, codecManager));
                FCTestThread.start();
                break;
            }
            case R.id.fullStartTestBtn:{
                visualLanding = new VisualLanding(codecManager);
                visualLanding.startVisualLanding();
                break;
            }
            case R.id.exitTestBtn:{
//                    ((Aircraft)RDApplication.getProductInstance()).getFlightController()
//                            .setVirtualStickModeEnabled(false, new CommonCallbacks.CompletionCallback() {
//                                @Override
//                                public void onResult(DJIError djiError) {
//                                    if(djiError != null) {
//                                        Tools.showToast(TestingActivity.this, "Exit error: "+djiError.getDescription());
////                                    }
////                                }
////                            });
                boolean done = false;
                if(visualLanding != null) {
                    if (visualLanding.visualLandingFlightControl != null) {
                        visualLanding.visualLandingFlightControl.endVisualLandingFlightControl();
                        done = true;
                    }
                }
                if(!done) {
                    flightController.setVirtualStickModeEnabled(false, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if(djiError != null) {
                                Log.d(TAG, djiError.getDescription());
                            }
                        }
                    });
                }
                break;
            }
            case R.id.multiTest1Btn:{
//                //pitch pid param adjust
//                getDataFromET();
//                EventBus.getDefault().post(new PIDParamChangeEvent(a, b, c, 1));
                codecManager.enabledYuvData(false);
                break;
            }
            case R.id.multiTest2Btn:{
                //roll pid param adjust
                getDataFromET();
                EventBus.getDefault().post(new PIDParamChangeEvent(a, b, c, 2));
                break;
            }
            case R.id.multiTest3Btn:{
                targetDetect = new TargetDetect(codecManager, this);
                targetDetection = new Thread(targetDetect);
                targetDetection.start();
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

    private void testTargetDetect() {

        targetDetect = new TargetDetect(this, codecManager.getVideoWidth(), codecManager.getVideoHeight());
        imageView.setVisibility(View.VISIBLE);
//        targetDetect.codecManager = new DJICodecManager();
//        targetDetect.codecManager = null;
//        targetDetect.codecManager = codecManager;
////        while(keep) {
//            targetDetect = new TargetDetect(codecManager, this);
//            if (targetDetection == null) {
//                targetDetection = new Thread(targetDetect);
//                targetDetection.start();
//            }
        
        //------------------------------------------------------
//        frame = targetDetect.getTestMat();
//        if (frame != null) {
//            if (frame.cols() > 0 && frame.rows() > 0) {
//                bitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
//                Utils.matToBitmap(frame, bitmap);
//            }
//        } else {
//            Log.e(TAG, "fail to create the bitmap");
//        }
//        imageView.setImageBitmap(bitmap);
//        imageView.setVisibility(View.VISIBLE);

//        }

        //------------------------------------------------------

//        Bitmap bitmap = videoSurface.getBitmap();
//        imageView.setImageBitmap(bitmap);
//        imageView.setVisibility(View.VISIBLE);

        //------------------------------------------------------
//        if(yuvFrame != null){
//            bitmap = Bitmap.createBitmap(yuvFrame.width(), frame.height(), Bitmap.Config.ARGB_8888);
//            imageView.setImageBitmap(bitmap);
//            imageView.setVisibility(View.VISIBLE);
//        }
        //------------------------------------------------------
//            yuvFrame = new Mat(codecManager.getVideoHeight(), codecManager.getVideoWidth(), CvType.CV_8UC1);
//            yuvFrame.put(0, 0, yuv);
//            if(yuvFrame != null) {
////                bitmap = Bitmap.createBitmap(yuvFrame.width(), yuvFrame.height(), Bitmap.Config.ARGB_8888);
//                bitmap = BitmapFactory.decodeByteArray(yuv, 0, yuv.length);
//                imageView.setImageBitmap(bitmap);
//                imageView.setVisibility(View.VISIBLE);
//            }

            //---------------------yuv2bitmap
//            long time1 = System.currentTimeMillis();
//            ByteArrayOutputStream out = new ByteArrayOutputStream();
//            YuvImage yuvImage = new YuvImage(yuv, ImageFormat.NV21, codecManager.getVideoWidth(), codecManager.getVideoHeight(),null);
//            yuvImage.compressToJpeg(new Rect(0, 0, codecManager.getVideoWidth(), codecManager.getVideoHeight()), 50, out);
//            byte[] imageBytes = out.toByteArray();
//            Bitmap image = BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.length);
//            imageView.setImageBitmap(image);
//            long time2 = System.currentTimeMillis();
//            long yuvDuration = time2-time1;
//            Log.d(TAG, "YUVImageDuration: "+yuvDuration);

            //----------------------
//            frame = targetDetect.yuvTest(yuv, codecManager.getVideoWidth(), codecManager.getVideoHeight());
//            if(frame != null){
//                Bitmap bitmap1 = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
//                imageView.setImageBitmap(bitmap1);
//                imageView.setVisibility(View.VISIBLE);
//            }
            //-----------------------yuv2mat
            Mat mat1 = targetDetect.yuvTest();
            bitmap = Bitmap.createBitmap(mat1.width(), mat1.height(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(mat1, bitmap);

            //-------------------------yuv detect
//            Mat de = targetDetect.yuvTest(yuv, w, h);
//            bitmap = Bitmap.createBitmap(de.width(), de.height(), Bitmap.Config.ARGB_8888);
//            Utils.matToBitmap(de, bitmap);
            imageView.setImageBitmap(bitmap);

    }

    int cnt = 0;
    long timeS, time1;
    private List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    int videoHeight, videoWidth, resizeH, resizeW;
    private void yuvTest(){
        videoHeight = codecManager.getVideoHeight();
        videoWidth = codecManager.getVideoWidth();
        resizeW = 200;
        resizeH = 200 * videoHeight / videoWidth;

        codecManager.enabledYuvData(true);
        codecManager.setYuvDataCallback(new DJICodecManager.YuvDataCallback() {
            @Override
            public void onYuvDataReceived(MediaFormat mediaFormat, ByteBuffer byteBuffer, int dataSize, int i1, int i2) {
                Log.d(TAG, "YUVDataReceivedStart");
                //-------------------------------------
                if(cnt % 10 == 0){
                    cnt = 0;
                    synchronized (this){

                        yuv = new byte[dataSize];
                        byteBuffer.get(yuv);

                        //-----------------------------
                        timeS = System.currentTimeMillis();
                        long time1 = timeS;

                        Mat yuvFrame = new Mat(videoHeight+videoHeight/2, videoWidth, CvType.CV_8UC1);
                        if(yuv == null){
                            Log.d(TAG, "detectReturn");
                        }
                        yuvFrame.put(0, 0, yuv);
                        if(yuvFrame == null || yuvFrame.empty()) {
                            Log.d(TAG, "detectReturn2");
                        }
                        Log.d(TAG, "yuvFrame: "+(System.currentTimeMillis()-timeS));
                        timeS = System.currentTimeMillis();

                        Mat frameTemp = new Mat(videoHeight, videoWidth, CvType.CV_8UC3);

                        //yuv 2 bgr
                        cvtColor(yuvFrame, frameTemp, COLOR_YUV2RGB_I420);

                        //resize
                        Mat frameDealing = new Mat(resizeH, resizeW, CvType.CV_8UC4);
                        resize(frameTemp, frameDealing, frameDealing.size(), 0, 0);
                        Log.d(TAG, "resize: "+(System.currentTimeMillis()-timeS));
                        timeS = System.currentTimeMillis();

                        //RGB 2 gray
                        cvtColor(frameDealing, frameDealing, COLOR_RGB2GRAY);
                        Log.d(TAG, "rgb2gray: "+(System.currentTimeMillis()-timeS));
                        timeS = System.currentTimeMillis();

                        //gray 2 binary frame
//        threshold(frameDealing, frameDealing, 200 ,250, THRESH_BINARY);   // ? enable threshold value for self-adaptation
                        adaptiveThreshold(frameDealing, frameDealing, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 15, 20);
                        Log.d(TAG, "binary: "+(System.currentTimeMillis()-timeS));
                        timeS = System.currentTimeMillis();

                        //get contours
                        findContours(frameDealing, contours, new Mat(), RETR_LIST, CHAIN_APPROX_NONE);

                        //get the contours index
                        int targetIndex = 0;
                        if(contours.size()>0) {
                            MatOfPoint target = contours.get(0);
                            double targetArea = contourArea(target);
                            double area, ratio, w, h;
                            RotatedRect rect;

                            for(int i=0; i<contours.size(); i ++) {

                                MatOfPoint temp = contours.get(i);

                                //area judge
                                area = contourArea(temp);
                                if(area < targetArea)
                                    continue;

                                //transfer
                                MatOfPoint2f temp2 = new MatOfPoint2f();
                                temp.convertTo(temp2, CvType.CV_32F);

                                //rect, w & h ratio judgement
                                rect = minAreaRect(temp2);
                                w = rect.size.width;
                                h = rect.size.height;
                                if(w!=0 && h!=0) {
                                    ratio = w/h;
                                }else
                                    continue;

                                if(Math.abs(ratio-1) < 0.1) {
                                    targetArea = area;
                                    targetIndex = i;
                                }
                            }
                        }
                        Log.d(TAG, "getIndex: "+(System.currentTimeMillis()-timeS));
                        timeS = System.currentTimeMillis();

                        //get center
                        Moments Moments = moments(contours.get(targetIndex));
                        PointF tp = new PointF((float) (Moments.m10 / Moments.m00)/resizeW, (float) (Moments.m01 / Moments.m00)/resizeH);

                        EventBus.getDefault().postSticky(new TargetPointResultEvent(tp));

                        Log.d(TAG, "theDetectedPointIs: "+tp);

                        Log.d(TAG, "detectionDuration:"+(System.currentTimeMillis()-time1));
                    }
                }
                cnt ++;
                //-------------------------------------
            }
        });
    }
//
//    A/libc: Fatal signal 6 (SIGABRT), code -6 in tid 3163 (RenderThread)
    private void getDataFromET() {
        if(pidPET.getText().toString() != null){
            a = Float.parseFloat(pidPET.getText().toString());
        }

        if(pidIET.getText().toString() != null) {
            b = Float.parseFloat(pidIET.getText().toString());
        }

        if(pidDET.getText().toString() != null) {
            c = Float.parseFloat(pidDET.getText().toString());
        }
        Tools.showToast(this, "Change the PID params.");
    }

    @Override
    public void surfaceCreated(SurfaceHolder surfaceHolder) {
        Log.d(TAG, "SurfaceViewStart");
        drawingLandingPointThread = new DrawingLandingPointThread(drawingSurface, videoSurface.getHeight(), videoSurface.getWidth());
        drawingLandingPointThread.start();
    }

    @Override
    public void surfaceChanged(SurfaceHolder surfaceHolder, int i, int i1, int i2) {

    }

    @Override
    public void surfaceDestroyed(SurfaceHolder surfaceHolder) {
        drawingLandingPointThread.drawingControl = false;
        drawingLandingPointThread.interrupt();
    }
}

