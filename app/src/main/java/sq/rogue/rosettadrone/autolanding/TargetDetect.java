package sq.rogue.rosettadrone.autolanding;

import static org.opencv.imgproc.Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_NONE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2GRAY;
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


import android.graphics.PointF;
import android.os.Build;
import android.util.Log;

import androidx.annotation.RequiresApi;

import org.greenrobot.eventbus.EventBus;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import dji.sdk.codec.DJICodecManager;

public class TargetDetect implements Runnable {

    private static final String TAG = "TargetDetection";

    private boolean isTargetPointAtCenter = false;
    public boolean exit = false;

    private float POINT_ERROR = 0.05f;

    private Mat frame = null;
    public Mat frameThreshold = null;      //testing frame
    private List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    private Point currentCenter;
    public Point targetPoint = null;
    public float targetRadius = 0;
    public TargetPointResultEvent targetPointResultEvent = new TargetPointResultEvent();

    public DJICodecManager codecManager;
    private int videoWidth = -1;
    private int videoHeight;
    private byte[] RGBAData = null;

    //!!?? extends from the parent Activity
    public TargetDetect(DJICodecManager codecManager) {
         this.codecManager = codecManager;
    }

    @Override
    public void run() {
        while (!isTargetPointAtCenter) {
            targetPointResultEvent.setPoint(getFlyPoint(), targetRadius);
            EventBus.getDefault().postSticky(targetPointResultEvent);
//            EventBus.getDefault().postSticky(new TargetPointResultEvent(targetPoint,));
            Log.d(TAG, "sendTargetPointResultEvent"+targetPointResultEvent.targetPoint);
        }
//        if(!exit) {
//            EventBus.getDefault().postSticky(new TargetAtCenterEvent());
//            Log.d(TAG, "sendTargetAtCenterEvent.");
//        }
    }

    //-------------------------Target Detection-------------------------
    //in invoked order

    public PointF getFlyPoint(){
        currentCenter = getTargetPoint();
        if(currentCenter == null) {
            return null;
        }
        int frameW = frame.width();
        int frameH = frame.height();

        double x = currentCenter.x/frameW;
        double y = currentCenter.y/frameH;

        //whether the point is in center
        if( (Math.abs(frameW - currentCenter.x) / frameW) < POINT_ERROR
            && (Math.abs(frameH - currentCenter.y) / frameH) < POINT_ERROR) {
            isTargetPointAtCenter = true;
            EventBus.getDefault().postSticky(new TargetAtCenterEvent());
        }

        float xF = ((float) x);
        float yF = ((float) y);
        return new PointF( xF, yF);
    }

     private Point getTargetPoint(){
        long time1 = System.currentTimeMillis();

        getVideoData();
        if(frame == null) {
            return null;
        }
        Mat frameDealing = new Mat();

        //RGB 2 gray
//        if(frame.empty()) {
//            Log.d(TAG, "frameDealingEmpty");
//            return null;
//        }

        cvtColor(frame, frameDealing, COLOR_RGB2GRAY);

        //gray 2 binary frame
//        threshold(frameDealing, frameDealing, 200 ,250, THRESH_BINARY);   // ? enable threshold value for self-adaptation
        adaptiveThreshold(frameDealing, frameDealing, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 15, 20);

        //testing
        frameThreshold = frameDealing;

        //CLOSE
//        Mat element = getStructuringElement(MORPH_RECT, new Size(7, 7));
//        morphologyEx(frameDealing, frameDealing, MORPH_CLOSE, element);

        //circle
//        Mat circle = new Mat();
//        HoughCircles(frameDealing, circle, HOUGH_GRADIENT, 2, frameDealing.width()/10, frameDealing.width()/2);
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
//        targetPoint = center;
//        if(targetPoint == null) {
//            return null;
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
                 if(w!=0 && h!=0) {
                     ratio = w/h;
                 }else{
                     continue;
                 }

                 if(Math.abs(ratio-1) < 0.1) {
                     double area = contourArea(temp);
                     if(area > targetArea) {
                         targetArea = area;
                         target = temp;
                         targetIndex = i;
                     }
                 }
             }
         }

         //get center
        Moments Moments = moments(contours.get(targetIndex));
        targetPoint = new Point(Moments.m10 / Moments.m00, Moments.m01 / Moments.m00);

        long time2 = System.currentTimeMillis();
        long timeDuration = time2 - time1;
        Log.d(TAG, " ProcessingDuration: " + timeDuration);

        return targetPoint;
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public Mat getTestMat(){
        long time1 = System.currentTimeMillis();

        getVideoData();
        if(frame == null) {
            return null;
        }

        Mat frameDealing = new Mat();

        //RGB 2 gray
        cvtColor(frame, frameDealing, COLOR_RGB2GRAY);

        //gray 2 binary frame
//        threshold(frameDealing, frameDealing, 210 ,255, THRESH_BINARY);   // ? enable threshold value for self-adaptation
        adaptiveThreshold(frameDealing, frameDealing, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 15, 15);

        //testing
        frameThreshold = frameDealing;

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

        double x = targetPoint.x/frameW;
        double y = targetPoint.y/frameH;

        //whether the point is in center
        if( (Math.abs(frameW - targetPoint.x) / frameW) < POINT_ERROR
                && (Math.abs(frameH - targetPoint.y) / frameH) < POINT_ERROR) {
            isTargetPointAtCenter = true;
            EventBus.getDefault().post(new TargetAtCenterEvent());
        }

//        float xF = ((float) x);
//        float yF = ((float) y);
//        PointF point = new PointF( xF, yF);
//        Log.d(TAG, "Target point: " + point);

        long time2 = System.currentTimeMillis();
        long timeDuration = time2-time1;
        Log.d(TAG, " ProcessingDuration: " + timeDuration);

//        EventBus.getDefault().postSticky(new TargetPointResultEvent(point, (float) targetRadius));

        return frame;
    }

    public Mat testingThresholdFrame () {
        getVideoData();
        if(frame == null) {
            return null;
        }

        Mat frameDealing = new Mat();

        //RGB 2 gray
        cvtColor(frame, frameDealing, COLOR_RGB2GRAY);

        //gray 2 binary frame
//        threshold(frameDealing, frameDealing, 210 ,255, THRESH_BINARY);   // ? enable threshold value for self-adaptation
        adaptiveThreshold(frameDealing, frameDealing, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 15, 15);

        return frameDealing;
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
        targetPoint = null;
        targetPoint = getTargetPoint();
        if(targetPoint != null) {
            return true;
        }
        return false;
    }

    //-------------------------Get Video Source-------------------------
    public void getVideoData() {
        videoWidth = codecManager.getVideoWidth();
        videoHeight = codecManager.getVideoHeight();
        RGBAData = codecManager.getRgbaData(videoWidth, videoHeight);
        if(RGBAData != null && RGBAData.length % 4 == 0) {
            Mat frameTemp = new Mat(videoHeight, videoWidth, CvType.CV_8UC4);
            frameTemp.put(0, 0, RGBAData);
            frame = new Mat(320*videoHeight/videoWidth, 320, CvType.CV_8UC4);
            resize(frameTemp, frame, frame.size(), 0, 0);
            if(!frame.empty()) {
                Log.d(TAG, "resizeSuccessfully");
            }
        }
    }
}
