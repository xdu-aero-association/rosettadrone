package sq.rogue.rosettadrone.autolanding;

import android.graphics.PointF;
import android.view.View;
import static org.opencv.imgproc.Imgproc.*;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Moments;
import org.opencv.videoio.VideoCapture;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.flighthub.model.LiveStream;
import sq.rogue.rosettadrone.DroneModel;


public class TargetDetecter {

    //for read in
    LiveStream liveStream = new LiveStream();
    String rtmpURL;                 //? not use
    VideoCapture capture;
    private int FrameCnt;
    private int MAX_DEALING_FRAMES = 30;

    //for image process
    private Mat frame = new Mat();
    private List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    private int MaxCircleNum;
    private double[] Circles;

    //for getting target center point
    private Point currentCenter;
    private Point previousValidCenter;
    boolean isCenterPointValid;
    private double MAX_DEVITATION = 0.01;
    View parent;

    public TargetDetecter(){
        rtmpURL = liveStream.getRtmpURL();
        capture = new VideoCapture(rtmpURL);
    }

    private Point getCurrentPoint(Mat frame){

        Mat GrayFrame = new Mat();
        Mat BinaryFrame = new Mat();
        Mat hierarchy = new Mat();
        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();

        /*
        Please use the target object as same as the one we recommend to get better image process.
         */

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

        //get center
        Moments mMoments = moments(contours.get(max));
        Point mMomentsP = new Point(mMoments.m10 / mMoments.m00, mMoments.m01 / mMoments.m00);

        return mMomentsP;
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

    private boolean pointIsCenter(Point previous, Point current){
        //
        if( (previous.x - current.x) / frame.width() > MAX_DEVITATION){
            return false;
        }
        return true;
    }

    public PointF getFlyPoint(){
        //return to TapFlyMission
        if(!capture.isOpened()){
            //? show "Livestream start with error, please try again or exit."
        }
        else{
            capture.read(frame);
            currentCenter = getCurrentPoint(frame);
        }
        double x = currentCenter.x/frame.size().width;
        double y = currentCenter.y/frame.size().height;
        float xF = ((float) x);
        float yF = ((float) y);
        return new PointF( xF, yF);
    }
}
