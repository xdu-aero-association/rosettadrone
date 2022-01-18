package sq.rogue.rosettadrone.autolanding;

import android.graphics.PointF;
import android.util.Log;

import static org.opencv.imgproc.Imgproc.*;

import androidx.annotation.Nullable;

import org.greenrobot.eventbus.EventBus;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Moments;
import org.opencv.videoio.VideoCapture;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.TimerTask;

import dji.common.camera.CameraVideoStreamSource;
import dji.common.error.DJIError;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.mission.tapfly.TapFlyExecutionState;
import dji.common.mission.tapfly.TapFlyMission;
import dji.common.mission.tapfly.TapFlyMissionState;
import dji.common.mission.tapfly.TapFlyMode;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.mission.tapfly.*;
import dji.sdk.sdkmanager.DJISDKManager;
import sq.rogue.rosettadrone.RDApplication;
import sq.rogue.rosettadrone.settings.GeneralUtils;
import sq.rogue.rosettadrone.settings.Tools;
import sq.rogue.rosettadrone.video.NativeHelper;

public class TargetDetect extends TimerTask implements Runnable {

    private static final String TAG = "TargetDetection";

    private boolean isTargetPointAtCenter = false;

    private int FRAME_SPACING = 5;
    private int FRAME_RESULT_VALID = 5;
    private int FRAME_IN_ONCE = 50;
    private float POINT_ERROR = 0.05f;

    private Mat frame = null;
    private List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    private Point currentCenter;
    public Point targetPoint = null;

    private VideoFeeder.VideoFeed videoFeed;
    public VideoFeeder.VideoDataListener videoDataListener;
    public DJICodecManager codecManager;
    private int videoWidth = -1;
    private int videoHeight;
    private byte[] RGBAData = null;

    public TargetDetect(DJICodecManager codecManager) {
         this.codecManager = codecManager;
    }

    public TargetDetect() {

    }

    @Override
    public void run() {
        videoFeed = VideoFeeder.getInstance().provideTranscodedVideoFeed();
        videoFeed.addVideoDataListener(new VideoFeeder.VideoDataListener() {
            @Override
            public void onReceive(byte[] videoBuffer, int size) {
                Log.d(TAG, "Video size:" + size);
            }
        });

        getVideoData();
        if(!isTargetPointAtCenter) {
            EventBus.getDefault().post(new TargetPointResultEvent(getFlyPoint(), isTargetInVision()));
        } else {
            EventBus.getDefault().post(new ThreadEvent());
        }

    }

    //-------------------------Target Detection-------------------------
    //in invoked order

    public PointF getFlyPoint(){
        currentCenter = getTargetPoint(frame);
        int frameW = frame.width();
        int frameH = frame.height();

        double x = currentCenter.x/frameW;
        double y = currentCenter.y/frameH;

        //whether the point is in center
        if( (Math.abs(frameW - currentCenter.x) / frameW) < POINT_ERROR
            && (Math.abs(frameH - currentCenter.y) / frameH) < POINT_ERROR) {
            isTargetPointAtCenter = true;
        }
        float xF = ((float) x);
        float yF = ((float) y);
        return new PointF( xF, yF);
    }

    protected Point getTargetPoint(Mat frame){

        Mat frameDealing = new Mat();

        //RGB 2 gray
        cvtColor(frame, frameDealing, COLOR_RGB2GRAY);

        //gray 2 binary frame
        threshold(frameDealing, frameDealing, 220 ,250, THRESH_BINARY);   // ? enable threshold value for self-adaptation

        //CLOSE
        Mat element = getStructuringElement(MORPH_RECT, new Size(7, 7));
        morphologyEx(frameDealing, frameDealing, MORPH_CLOSE, element);

        //get contours
        findContours(frameDealing, contours, new Mat(), RETR_LIST, CHAIN_APPROX_NONE);
        connectedComponentsWithStats(frameDealing, new Mat(), new Mat(), new Mat());
        getBiggestContours(contours);
        int max = getBiggestContoursNumber(contours);
        if(max == 0) {
            Log.d(TAG, "There is no contour detected in the frame.");
            return null;
        }

        //get center
        Moments Moments = moments(contours.get(max));
        targetPoint = new Point(Moments.m10 / Moments.m00, Moments.m01 / Moments.m00);

        //visualizing the result
        drawContours(frame, contours, max, new Scalar(0, 0, 255) ,FILLED);
        circle(frame, targetPoint,4, new Scalar(0, 0, 255), FILLED);

        return targetPoint;
    }

//!!??merge?
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

    private boolean isTargetInVision() {
//!!??
        int spacingFrame;
        int validFrame = 0;
        int countFrameInOnce = 0;

        while(countFrameInOnce != FRAME_IN_ONCE) {
            getVideoData();
            spacingFrame = 0;
            while(spacingFrame != FRAME_SPACING) {
                if(getTargetPoint(frame) != null) {
                    validFrame ++;
                }
                spacingFrame ++;
                countFrameInOnce ++;
            }
        }

        if(validFrame > FRAME_RESULT_VALID) {
            return true;
        }
        return false;
    }

    //-------------------------Get Video Source-------------------------
    public int getVideoData() {
        videoWidth = codecManager.getVideoWidth();
        videoHeight = codecManager.getVideoHeight();
        RGBAData = codecManager.getRgbaData(videoWidth, videoHeight);
        frame = new Mat(videoHeight, videoWidth, CvType.CV_8UC4);
        frame.put(0, 0, RGBAData);
        if(frame == null) {
            Log.d(TAG, "The Mat frame is null!");
        }
        return videoWidth;
    }
}
