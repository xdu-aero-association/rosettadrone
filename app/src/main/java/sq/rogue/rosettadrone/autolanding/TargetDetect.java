package sq.rogue.rosettadrone.autolanding;

import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_NONE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2GRAY;
import static org.opencv.imgproc.Imgproc.FILLED;
import static org.opencv.imgproc.Imgproc.MORPH_CLOSE;
import static org.opencv.imgproc.Imgproc.MORPH_RECT;
import static org.opencv.imgproc.Imgproc.RETR_LIST;
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
import android.graphics.PointF;
import android.util.Log;

import org.greenrobot.eventbus.EventBus;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;

public class TargetDetect implements Runnable {

    private static final String TAG = "TargetDetection";

    private boolean isTargetPointAtCenter = false;
    private Activity parent;

    private float POINT_ERROR = 0.05f;

    private Mat frame = null;
    private List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    private Point currentCenter;
    public Point targetPoint = null;

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
        if(!isTargetPointAtCenter) {
            EventBus.getDefault().post(new TargetPointResultEvent(getFlyPoint(), isTargetInVision()));
        } else {
            EventBus.getDefault().post(new TargetAtCenterEvent());
        }
    }

    //-------------------------Target Detection-------------------------
    //in invoked order

    public PointF getFlyPoint(){
        currentCenter = getTargetPoint();
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

    protected Point getTargetPoint(){

        getVideoData();
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

    public Mat getTestMat(){

        getVideoData();

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

        return frame;
    }

    private int getBiggestContoursNumber(List<MatOfPoint> contours) {
        int max = 0;
        if(contours != null) {
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
        frame = new Mat(videoHeight, videoWidth, CvType.CV_8UC4);
        frame.put(0, 0, RGBAData);
        if(frame == null) {
            Log.d(TAG, "The Mat frame is null!");
        }
    }
}
