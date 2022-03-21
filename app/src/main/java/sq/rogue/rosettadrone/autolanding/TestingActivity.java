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
    public void onClick(View v){
        switch (v.getId()) {
            case R.id.targetAndGimbalTestBtn:{
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
                codecManager.enabledYuvData(false);
                break;
            }
            case R.id.multiTest2Btn:{
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

        imageView.setVisibility(View.VISIBLE);
        Mat mat1 = targetDetect.yuvTest();
        bitmap = Bitmap.createBitmap(mat1.width(), mat1.height(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(mat1, bitmap);
        imageView.setImageBitmap(bitmap);
    }

    int cnt = 0;
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
            public void onYuvDataReceived(MediaFormat mediaFormat, ByteBuffer byteBuffer, int i, int i1, int i2) {
                if(cnt % 5 == 0) {
                    yuv = new byte[i];
                    byteBuffer.get(yuv);
                    cnt = 0;
                }
                cnt ++;
            }
        });
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

