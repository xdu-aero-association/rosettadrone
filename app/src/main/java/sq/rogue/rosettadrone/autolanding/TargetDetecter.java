package sq.rogue.rosettadrone.autolanding;

import static org.opencv.imgproc.Imgproc.COLOR_RGB2GRAY;
import static org.opencv.imgproc.Imgproc.HOUGH_GRADIENT;
import static org.opencv.imgproc.Imgproc.HoughCircles;
import static org.opencv.imgproc.Imgproc.MORPH_CLOSE;
import static org.opencv.imgproc.Imgproc.MORPH_RECT;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.getStructuringElement;
import static org.opencv.imgproc.Imgproc.morphologyEx;
import static org.opencv.imgproc.Imgproc.threshold;

import android.graphics.PointF;
import android.view.View;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.videoio.VideoCapture;

import java.util.List;

import dji.sdk.flightcontroller.FlightController;
import dji.sdk.flighthub.model.LiveStream;
import sq.rogue.rosettadrone.DroneModel;


public class TargetDetecter implements Runnable{


    //for read in
    LiveStream liveStream = new LiveStream();
    String rtmpURL;
    VideoCapture capture;
    private int FrameCnt;
    private int MAX_DEALING_FRAMES = 30;

    //for image process
    private Mat frame = new Mat();
    private List<MatOfPoint> contours;
    private int MaxCircleNum;
    private double[] Circles;

    //for getting target center point
    private Point currentCenter;
    private Point previousValidCenter;
    boolean isCenterPointValid;
    private double MAX_DEVITATION = 0.01;
    View parent;

    //for fly control
    private DroneModel mModel;
    private FlightController mFlightController;
    private double droneLocationLat;
    private double droneLocationLng;



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

        //1\ get contours
//        findContours(BinaryFrame, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);
//        connectedComponentsWithStats(BinaryFrame, labels, stats, centroids);

        //2\ get the contours according to shape and area
        Mat CircleFrame = new Mat();
        HoughCircles(BinaryFrame, CircleFrame, HOUGH_GRADIENT, 1, 10);

        //get the center point
        double MaxRadius = CircleFrame.get(0, 0)[2];
        for(int i=0; i< CircleFrame.cols(); i++){
            Circles = CircleFrame.get(0, i);
            if(MaxRadius < Circles[2]) {
                MaxRadius = Circles[2];
                MaxCircleNum = i;
            }
        }
        Circles = CircleFrame.get(0, MaxCircleNum);

        return new Point(Circles[0], Circles[1]);
    }

    private boolean pointIsValid(Point previous, Point current){
        if( (previous.x - current.x) / frame.width() > MAX_DEVITATION){
            return false;
        }
        return true;
    }

    public PointF getFlyPoint(){
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


    @Override
    public void run() {

        //? read in
        //way 1: through DJI's livestreamer to get rtsp url
        //way 2: thourgh opencv's videocapture(1) method
        //way 3: through opencv's camera class extend from java module

        if(!capture.isOpened()){
            //? show "Livestream start with error, please try again or exit."
        }
        else{
            FrameCnt = 0;
            boolean FirstStart = true;
            while(capture.read(frame)){
                //select frames for speed up
                if(FrameCnt != MAX_DEALING_FRAMES){
                    FrameCnt++;
                }
                else{
                    FrameCnt = 0;

                    currentCenter = getCurrentPoint(frame);

                    if(FirstStart){
                        previousValidCenter = currentCenter;
                    }
                    isCenterPointValid = pointIsValid(previousValidCenter, currentCenter);

                    if(isCenterPointValid){

                    }
                }
            }
        }
    }

}
