package sq.rogue.rosettadrone.autolanding;

import android.graphics.PointF;

import static org.opencv.imgproc.Imgproc.*;

import androidx.annotation.Nullable;

import org.greenrobot.eventbus.EventBus;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Moments;
import org.opencv.videoio.VideoCapture;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

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

public class TargetDetecter implements Runnable {

    //for image process
    private Mat frame = new Mat();
    private List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    private int FRAME_SPACING = 5;
    private int FRAME_RESULT_VALID = 5;
    private int FRAME_IN_ONCE = 50;

    //for getting target center point
    private Point currentCenter;
    private double MAX_DEVITATION = 0.01;
    private static final float MIN_HEIGHT = 30;

    //for getting video src
    private BaseProduct product;
    private Camera camera;
    private VideoFeeder.VideoFeed videoFeed;
    public VideoFeeder.VideoDataListener videoDataListener;
    private DJICodecManager codecManager;

    public TargetDetecter(){
        product = RDApplication.getProductInstance();
        if(product == null) {

        } else{

        }
        camera = RDApplication.getProductInstance().getCamera();
        if (camera != null){
            // Reset the callback
            VideoFeeder.getInstance().getPrimaryVideoFeed().removeVideoDataListener(videoDataListener);
            camera.setCameraVideoStreamSourceCallback(new CameraVideoStreamSource.Callback() {
                @Override
                public void onUpdate(CameraVideoStreamSource cameraVideoStreamSource) {

                }
            });
        }
    }

    @Override
    public void run() {
        while(true){
            EventBus.getDefault().post(new TargetPointResultEvent(getFlyPoint(), isTargetInVision()));
        }
        //receiver: firstStageController flight control
    }

    //-------------------------Target Detection-------------------------

    public PointF getFlyPoint(){
        //used in TapFlyMission directly
        getVideoData(frame);
        currentCenter = getTargetPoint(frame);
            //???official
        double x = currentCenter.x/frame.size().width;
        double y = currentCenter.y/frame.size().height;
        float xF = ((float) x);
        float yF = ((float) y);
        return new PointF( xF, yF);
    }

    private Point getTargetPoint(Mat frame){

        Mat GrayFrame = new Mat();
        Mat BinaryFrame = new Mat();
        Mat hierarchy = new Mat();
        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();

        //RGB 2 gray
        cvtColor(frame, GrayFrame, COLOR_RGB2GRAY);

        //gray 2 binary frame
        threshold(GrayFrame, BinaryFrame, 220 ,250, THRESH_BINARY);   // ? enable threshold value for self-adaptation

        //CLOSE
        Mat element = getStructuringElement(MORPH_RECT, new Size(7, 7));
        morphologyEx(BinaryFrame, BinaryFrame, MORPH_CLOSE, element);

        //get contours
        findContours(BinaryFrame, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);
        connectedComponentsWithStats(BinaryFrame, labels, stats, centroids);
        getBiggestContours(contours);
        int max = getBiggestContoursNumber(contours);
        if(max == 0) {
            return null;
        }

        //get center
        Moments Moments = moments(contours.get(max));
        Point targetPoint = new Point(Moments.m10 / Moments.m00, Moments.m01 / Moments.m00);

        return targetPoint;
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

    private boolean isTargetInVision() {

        int spacingFrame;
        int validFrame = 0;
        int countFrameInOnce = 0;

        while(countFrameInOnce != FRAME_IN_ONCE) {
            getVideoData(frame);
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


//    private boolean pointIsCenter(Point previous, Point current){
//        //
//        if( (previous.x - current.x) / frame.width() > MAX_DEVITATION){
//            return false;
//        }
//        return true;
//    }

//    private boolean isTargetPointValid() {
//
//        return true;
//    }


    //-------------------------Get Video Source-------------------------
    private void getVideoData(Mat frame) {
            //???
        videoDataListener = new VideoFeeder.VideoDataListener() {
            @Override
            public void onReceive(byte[] videoBuffer, int size) {
                if(codecManager != null){
                    codecManager.sendDataToDecoder(videoBuffer, size);
                }
                frame.put(100, 100, videoBuffer);
            }
        };
    }

}
