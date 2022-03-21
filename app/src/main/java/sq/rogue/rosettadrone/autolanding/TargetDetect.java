package sq.rogue.rosettadrone.autolanding;

import static org.opencv.imgproc.Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_NONE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2GRAY;
import static org.opencv.imgproc.Imgproc.COLOR_YUV2BGR_I420;
import static org.opencv.imgproc.Imgproc.COLOR_YUV2BGR_NV12;
import static org.opencv.imgproc.Imgproc.COLOR_YUV2RGB_I420;
import static org.opencv.imgproc.Imgproc.FILLED;
import static org.opencv.imgproc.Imgproc.HoughCircles;
import static org.opencv.imgproc.Imgproc.MORPH_CLOSE;
import static org.opencv.imgproc.Imgproc.MORPH_RECT;
import static org.opencv.imgproc.Imgproc.RETR_LIST;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY_INV;
import static org.opencv.imgproc.Imgproc.adaptiveThreshold;
import static org.opencv.imgproc.Imgproc.circle;
import static org.opencv.imgproc.Imgproc.connectedComponentsWithStats;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.getStructuringElement;
import static org.opencv.imgproc.Imgproc.minAreaRect;
import static org.opencv.imgproc.Imgproc.moments;
import static org.opencv.imgproc.Imgproc.morphologyEx;
import static org.opencv.imgproc.Imgproc.resize;


import android.graphics.Bitmap;
import android.graphics.PointF;
import android.graphics.SurfaceTexture;
import android.media.MediaFormat;
import android.os.Build;
import android.util.Log;

import androidx.annotation.RequiresApi;

import org.greenrobot.eventbus.EventBus;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Moments;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import dji.sdk.codec.DJICodecManager;

public class TargetDetect implements Runnable {

    private static final String TAG = "TargetDetection";

    private boolean isTargetPointAtCenter = false;
    public boolean exit = false;

    private float POINT_ERROR = 0.05f;

    private Mat frame = null;     //testing frame
    private List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    private PointF currentCenter;
    public Point targetPoint = null;
    public float targetRadius = 0;
    public TargetPointResultEvent targetPointResultEvent = new TargetPointResultEvent();

    public DJICodecManager codecManager;
    private int videoWidth;
    private int videoHeight;
    private byte[] RGBAData = null;
    protected VisualLandingFlightControl visualLandingFlightControl = null;
    protected TestingActivity testingActivity = null;
    private byte[] yuv = null;
    private int resizeW;
    private int resizeH;

    //!!?? extends from the parent Activity
    public TargetDetect(DJICodecManager codecManager) {
         this.codecManager = codecManager;
    }

    public TargetDetect(TestingActivity testingActivity,int width, int height){
        this.videoWidth = width;
        this.videoHeight = height;
        this.testingActivity = testingActivity;
        this.codecManager = testingActivity.codecManager;
    }

    public TargetDetect(DJICodecManager djiCodecManager, TestingActivity testingActivity){
        this.codecManager = djiCodecManager;
        this.videoWidth = codecManager.getVideoWidth();
        this.videoHeight = codecManager.getVideoHeight();
        this.testingActivity = testingActivity;
    }

    @Override
    public void run() {
        resizeW = 280;
        resizeH = 280 * videoHeight / videoWidth;
        while (true) {
            if(testingActivity.yuv != null) {
                targetPointResultEvent.setPoint(getTargetPoint(testingActivity.yuv), targetRadius);
                EventBus.getDefault().postSticky(targetPointResultEvent);
                Log.d(TAG, "sendTargetPointResultEvent" + targetPointResultEvent.targetPoint);
            }
        }
    }

    //-------------------------Target Detection-------------------------

    long timeS = 0;
    protected synchronized PointF getTargetPoint(byte[] yuvData){

        timeS = System.currentTimeMillis();
        long time1 = timeS;

        Mat yuvFrame = new Mat(videoHeight+videoHeight/2, videoWidth, CvType.CV_8UC1);
        if(yuvData == null){
            Log.d(TAG, "detectReturn");
            return null;
        }
        yuvFrame.put(0, 0, yuvData);
        if(yuvFrame == null || yuvFrame.empty()) {
            Log.d(TAG, "detectReturn2");
            return null;
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
        contours.clear();
        findContours(frameDealing, contours, new Mat(), RETR_LIST, CHAIN_APPROX_NONE);

         //get the contours index
         int targetIndex = 0;
         if(contours.size()>0) {
             MatOfPoint target = contours.get(0);
             double targetArea = contourArea(target);
             double area, ratio, w, h;
             RotatedRect rect;

             for(int i=0; i<contours.size(); i ++) {
                 //transfer
                 MatOfPoint temp = contours.get(i);
                 MatOfPoint2f temp2 = new MatOfPoint2f();
                 temp.convertTo(temp2, CvType.CV_32F);

                 //area judge
                 area = contourArea(temp);
                 if(area < targetArea)
                        continue;;
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
        Log.d(TAG, "getIndex: "+(System.currentTimeMillis()-timeS)+" contours size: " +contours.size());
        timeS = System.currentTimeMillis();

        //get center
        Moments Moments = moments(contours.get(targetIndex));
        PointF tp = new PointF((float) (Moments.m10 / Moments.m00)/resizeW, (float) (Moments.m01 / Moments.m00)/resizeH);

        Log.d(TAG, "theDetectedPointIs: "+tp);
        Log.d(TAG, "detectionDuration:"+(System.currentTimeMillis()-time1));

        return tp;
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public synchronized Mat getTestMat(){
        try {
            this.wait(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        getVideoData();
//        long time1 = System.currentTimeMillis();
//        Bitmap bitmap = getBitmap();
//        long time2 = System.currentTimeMillis();
//        long duration = time2-time1;
//        Log.d(TAG, "GetBitmap: "+duration);
//
//        frame = new Mat(bitmap.getHeight(), bitmap.getWidth(), CvType.CV_8UC4);
//
//        if(bitmap != null) {
//            Utils.bitmapToMat(bitmap, frame);
//        }else{
//            frame = null;
//        }
        if(frame == null) {
            return null;
        }

        Mat frameDealing = new Mat();

        //RGB 2 gray
        cvtColor(frame, frameDealing, COLOR_RGB2GRAY);

        //gray 2 binary frame
//        threshold(frameDealing, frameDealing, 210 ,255, THRESH_BINARY);   // ? enable threshold value for self-adaptation
        adaptiveThreshold(frameDealing, frameDealing, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 15, 15);
//        if(frameDealing!=null)
//            return frameDealing;

        //CLOSE
//        Mat element = getStructuringElement(MORPH_RECT, new Size(7, 7));
//        morphologyEx(frameDealing, frameDealing, MORPH_CLOSE, element);

//        //circle
//        Mat circle = new Mat();
//        HoughCircles(frameDealing, circle, HOUGH_GRADIENT, 2, frameDealing.width()/10);
//
//        Point center = null;
//        maxRadius = 0;
//        for(int i=0; i < circle.cols(); i++) {
//            double[] infoCircle = circle.get(0, i);
//            if(infoCircle[2] > maxRadius) {
//                if(center == null) {
//                    center = new Point(infoCircle[0], infoCircle[1]);
//                }else {
//                    center.x = infoCircle[0];
//                    center.y = infoCircle[1];
//                }
//
//                maxRadius = infoCircle[2];
//            }
//        }
        

        //get contours
        findContours(frameDealing, contours, new Mat(), RETR_LIST, CHAIN_APPROX_NONE);
//        connectedComponentsWithStats(frameDealing, new Mat(), new Mat(), new Mat());
//        getBiggestContours(contours);
//        int max = getBiggestContoursNumber(contours);
//        if(max == 0) {
//            Log.d(TAG, "There is no contour detected in the frame.");
//            return null;
//        }

        //get the contours index
        int targetIndex = 0;
        if(contours.size()>0) {
            MatOfPoint target = contours.get(0);
            double targetArea = contourArea(target);

            for(int i=0; i<contours.size(); i ++) {
                //transfer
                MatOfPoint temp = contours.get(i);
                MatOfPoint2f temp2 = new MatOfPoint2f();
                temp.convertTo(temp2, CvType.CV_32F);

                //rect, w & h ratio judgement
                RotatedRect rect = minAreaRect(temp2);
                double ratio = 0;
                double w = rect.size.width;
                double h = rect.size.height;
                float radius = 0;
                if(w!=0 && h!=0) {
                    ratio = w/h;
                    radius = (float) w/2;
                }else{
                    continue;
                }

                if(Math.abs(ratio-1) < 0.1) {
                    double area = contourArea(temp);
                    if(area > targetArea) {
                        targetArea = area;
                        target = temp;
                        targetIndex = i;
                        targetRadius = radius;
                    }
                }
            }
        }

        //get center
        Moments Moments = moments(contours.get(targetIndex));
        targetPoint = new Point(Moments.m10 / Moments.m00, Moments.m01 / Moments.m00);

        //visualizing the result
        drawContours(frame, contours, targetIndex, new Scalar(0, 0, 255) ,FILLED);
//        circle(frame, targetPoint, (int)targetRadius,  new Scalar(0, 255, 0), -1);
//        targetPoint = center;
//        if(targetPoint == null) {
//            return null;
//        }

//        circle(frame, targetPoint, (int) targetRadius, new Scalar(0, 0, 255), FILLED);

        int frameW = frame.width();
        int frameH = frame.height();

        float x = (float) targetPoint.x/frameW;
        float y = (float) targetPoint.y/frameH;

        //whether the point is in center
//        if( (Math.abs(frameW - targetPoint.x) / frameW) < POINT_ERROR
//                && (Math.abs(frameH - targetPoint.y) / frameH) < POINT_ERROR) {
//            isTargetPointAtCenter = true;
//            EventBus.getDefault().post(new TargetAtCenterEvent());
//        }

//        float xF = ((float) x);
//        float yF = ((float) y);
//        PointF point = new PointF( xF, yF);
//        Log.d(TAG, "Target point: " + point);

//        long time2 = System.currentTimeMillis();
//        long timeDuration = time2-time1;
//        Log.d(TAG, " ProcessingDuration: " + timeDuration);

//        EventBus.getDefault().postSticky(new TargetPointResultEvent(point, (float) targetRadius));

        PointF pointF = new PointF(x, y);
//        Log.d(TAG,"getFrameBitmap: " + pointF);
        return frame;
    }

    private int getBiggestContoursNumber(List<MatOfPoint> contours) {
        int max = 0;
        if(contours.size() > 0) {
            double max_area = contourArea(contours.get(0));
            for (int i = 0; i < contours.size(); i++) {
                double area = contourArea((contours.get(i)));
                if (area > max_area) {
                    max_area = area;
                    max = i;
                }
            }
        }
        return max;
    }

    private void getBiggestContours(List<MatOfPoint> contours) {
        if(contours.size() > 0) {
            Iterator<MatOfPoint> each = contours.iterator();
            MatOfPoint wrapper = contours.get(0);
            double maxArea = contourArea(wrapper);
            while (each.hasNext()) {
                wrapper = each.next();
                double area = contourArea(wrapper);
                if (area < maxArea) {
                    each.remove();
                }
            }
        }
    }

    public boolean isTargetInVision() {
        if(targetPoint != null) {
            return true;
        }
        return false;
    }

    //-------------------------Get Video Source-------------------------

    public void getVideoData() {
//        long time1 = System.currentTimeMillis();
//        this.codecManager = visualLandingFlightControl.djiCodecManager;
//        codecManager = null;
//        codecManager = testingActivity.codecManager;

        videoWidth = testingActivity.codecManager.getVideoWidth();
        videoHeight = testingActivity.codecManager.getVideoHeight();
        synchronized (this) {
            for(int i=0; i<20; i++) {
                long time2 = System.currentTimeMillis();
                RGBAData = new byte[videoWidth * videoHeight * 4];
                RGBAData = codecManager.getRgbaData(videoWidth, videoHeight);
                long time3 = System.currentTimeMillis();
                long partialDuration = time3 - time2;
                Log.d(TAG, "getFrameDuration(One): " + partialDuration);
            }
            if (RGBAData != null) {
                if (RGBAData.length % 4 == 0 || RGBAData.length > 0) {
                    Mat frameTemp = new Mat(videoHeight, videoWidth, CvType.CV_8UC4);
                    frameTemp.put(0, 0, RGBAData);
                    RGBAData = null;
                    frame = new Mat(320 * videoHeight / videoWidth, 320, CvType.CV_8UC4);
                    resize(frameTemp, frame, frame.size(), 0, 0);
                }
            }
        }
//        codecManager.destroyCodec();
//        long time4 = System.currentTimeMillis();
//            long wholeDuration = time4 - time1;
//            Log.d(TAG, "getFrameDuration(Whole): " + wholeDuration);
    }

    //test for yuv
    public synchronized Mat yuvTest(){

        resizeW = 200;
        resizeH = 200 * videoHeight / videoWidth;

        long time1 = System.currentTimeMillis();
        Mat yuvFrame = new Mat(videoHeight+videoHeight/2, videoWidth, CvType.CV_8UC1);
        if(testingActivity.yuv == null){
            return null;
        }
        yuvFrame.put(0, 0, testingActivity.yuv);
        if(yuvFrame == null || yuvFrame.empty()) {
            return null;
        }

        Mat frameTemp = new Mat(videoHeight, videoHeight, CvType.CV_8UC3);

        //yuv 2 bgr
        cvtColor(yuvFrame, frameTemp, COLOR_YUV2RGB_I420);

        //resize
        Mat frameDealing = new Mat(resizeH, resizeW, CvType.CV_8UC4);
        Mat frameOri = new Mat(resizeH, resizeW, CvType.CV_8UC4);
        resize(frameTemp, frameDealing, frameDealing.size(), 0, 0);
        resize(frameTemp, frameOri, frameOri.size(), 0, 0);
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
                //transfer
                MatOfPoint temp = contours.get(i);
                MatOfPoint2f temp2 = new MatOfPoint2f();
                temp.convertTo(temp2, CvType.CV_32F);

                //area judge
                area = contourArea(temp);
                if(area < targetArea)
                    continue;;
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
        drawContours(frameOri, contours, targetIndex, new Scalar(0, 0, 255) ,FILLED);
        Log.d(TAG, "detectionDuration:"+(System.currentTimeMillis()-time1));

        return frameOri;
    }
}
