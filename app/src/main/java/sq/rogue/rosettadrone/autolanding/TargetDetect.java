package sq.rogue.rosettadrone.autolanding;

import static org.opencv.core.CvType.CV_32F;
import static org.opencv.imgproc.Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_NONE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2GRAY;
import static org.opencv.imgproc.Imgproc.COLOR_YUV2RGB_I420;
import static org.opencv.imgproc.Imgproc.FILLED;
import static org.opencv.imgproc.Imgproc.RETR_LIST;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY_INV;
import static org.opencv.imgproc.Imgproc.adaptiveThreshold;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.minAreaRect;
import static org.opencv.imgproc.Imgproc.moments;
import static org.opencv.imgproc.Imgproc.resize;

import android.graphics.PointF;
import android.util.Log;

import org.greenrobot.eventbus.EventBus;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Moments;
import org.opencv.video.KalmanFilter;

import java.util.ArrayList;
import java.util.List;

import dji.sdk.codec.DJICodecManager;

public class TargetDetect implements Runnable {

    private static final String TAG = "TargetDetection";

    private boolean isTargetPointAtCenter = false;
    public boolean exit = false;

    private float POINT_ERROR = 0.05f;

    private Mat frame = null;     //testing frame
    private List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    public PointF targetPoint = null;
    public float targetRadius = 0;
    public TargetPointResultEvent targetPointResultEvent = new TargetPointResultEvent();

    public DJICodecManager codecManager;
    private int videoWidth;
    private int videoHeight;
    protected TestingActivity testingActivity = null;
    private byte[] yuv = null;
    private int resizeW;
    private int resizeH;

    private KalmanFilter kf;
    private Mat transitionMatrix;
    private Mat measurementMatrix;
    private Mat statePre;
    private Mat processNoiseCov;
    private Mat measurementNoiseCov;
    private Mat errorCovPost;


    public TargetDetect(TestingActivity testingActivity,int width, int height){
        this.videoWidth = width;
        this.videoHeight = height;
        this.testingActivity = testingActivity;
        this.codecManager = testingActivity.codecManager;
    }

    @Override
    public void run() {
        resizeW = 360;
        resizeH = 360 * videoHeight / videoWidth;
        kalmanInit();
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
    PointF preTargetPoint = null;
    protected synchronized PointF getTargetPoint(byte[] yuvData){

        timeS = System.currentTimeMillis();
        long time1 = timeS;

        contours.clear();

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
                 temp.convertTo(temp2, CV_32F);

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
        if(contours.size() == 0) {
            return null;
        }
        Moments Moments = moments(contours.get(targetIndex));
        targetPoint = new PointF((float) (Moments.m10 / Moments.m00)/resizeW, (float) (Moments.m01 / Moments.m00)/resizeH);

        //
        if(preTargetPoint != null) {
            if(Math.abs(targetPoint.x - preTargetPoint.x) > 0.45f || Math.abs(targetPoint.y - preTargetPoint.y) > 0.4f) {
                targetPoint.x = (float) (preTargetPoint.x * 0.7 + targetPoint.x * 0.3);
                targetPoint.y = (float) (preTargetPoint.y * 0.7 + targetPoint.y * 0.3);
            }
        }

        //kalman
//        kalmanWork();

        Log.d(TAG, "theDetectedPointIs: "+targetPoint);
        Log.d(TAG, "detectionDuration:"+(System.currentTimeMillis()-time1));

        preTargetPoint = targetPoint;

        return targetPoint;
    }

    private void kalmanInit() {

        //init
        kf = new KalmanFilter(2, 2, 0, CV_32F);

        Log.d(TAG, "CHECKING KalmanFilter Matrix: " +
                "\n transitionMatrix: " + kf.get_transitionMatrix().size() +
                "\n measurementMatrix: " + kf.get_measurementMatrix().size() +
                "\n statePre: " + kf.get_statePre().size() +
                "\n processionNoiseCov: " + kf.get_processNoiseCov().size() +
                "\n measurementNoiseCov: " + kf.get_measurementNoiseCov().size() +
                "\n errorCovPost: " + kf.get_errorCovPost().size());

        kalmanMatrixInit2();
    }

    private void kalmanMatrixInit4() {
//        CHECKING KalmanFilter Matrix:
//     transitionMatrix: 4x4
//     measurementMatrix: 4x2
//     statePre: 1x4
//     processionNoiseCov: 4x4
//     measurementNoiseCov: 2x2
//     errorCovPost: 4x4
        transitionMatrix = Mat.eye(4, 4, CV_32F);
        transitionMatrix.put(4, 4, new float[]{1, 0, 1, 0,
                0, 1, 0, 1,
                0, 0, 1, 0,
                0, 0, 0, 1});
        kf.set_transitionMatrix(transitionMatrix);

        measurementMatrix = Mat.eye(2, 4, CV_32F);
        kf.set_measurementMatrix(measurementMatrix);

        statePre = new Mat(4,1, CV_32F);
        statePre.put(0, 0, 0.5f);
        statePre.put( 1,0, 0.5f);
        statePre.put( 2, 0,0);
        statePre.put( 3,0, 0);
        kf.set_statePre(statePre);

        processNoiseCov = Mat.eye(4, 4, CV_32F);
        processNoiseCov = processNoiseCov.mul(processNoiseCov, 1e-1);
        kf.set_processNoiseCov(processNoiseCov);

        measurementNoiseCov = Mat.eye(2, 2, CV_32F);
        measurementNoiseCov = measurementNoiseCov.mul(measurementNoiseCov, 1e-1);
        kf.set_measurementNoiseCov(measurementNoiseCov);

        errorCovPost = Mat.eye(4, 4, CV_32F);
        errorCovPost = errorCovPost.mul(errorCovPost, 0.1);
        kf.set_errorCovPost(errorCovPost);
    }

    private void kalmanMatrixInit2() {
        //        CHECKING KalmanFilter Matrix:
//        transitionMatrix: 2x2
//        measurementMatrix: 2x2
//        statePre: 1x2
//        processionNoiseCov: 2x2
//        measurementNoiseCov: 2x2
//        errorCovPost: 2x2
        transitionMatrix = Mat.eye(2, 2, CV_32F);
        kf.set_transitionMatrix(transitionMatrix);

        measurementMatrix = Mat.eye(2, 2, CV_32F);
        kf.set_measurementMatrix(measurementMatrix);

        statePre = new Mat(2,1, CV_32F);
        statePre.put(0, 0, 0.5f);
        statePre.put( 1,0, 0.5f);
        kf.set_statePre(statePre);

        processNoiseCov = Mat.eye(2, 2, CV_32F);
        processNoiseCov = processNoiseCov.mul(processNoiseCov, 1e-1);
        kf.set_processNoiseCov(processNoiseCov);

        measurementNoiseCov = Mat.eye(2, 2, CV_32F);
        measurementNoiseCov = measurementNoiseCov.mul(measurementNoiseCov, 1e-1);
        kf.set_measurementNoiseCov(measurementNoiseCov);

        errorCovPost = Mat.eye(2, 2, CV_32F);
        errorCovPost = errorCovPost.mul(errorCovPost, 0.1);
        kf.set_errorCovPost(errorCovPost);
    }

    private void kalmanWork() {
        //kalman

        //update
        kf.get_measurementMatrix().put(0, 0, targetPoint.x*videoWidth);
        kf.get_measurementMatrix().put(1, 0, targetPoint.y*videoWidth);

        Mat measure = new Mat(2, 1, CV_32F);
        measure.put(0, 0, targetPoint.x*videoWidth);
        measure.put(1, 0, targetPoint.y*videoHeight);

        //correct
        Mat test = kf.correct(measure);
        Log.d(TAG, "correct: " + test.size());

        //predict
        Mat prediction = kf.predict();
        if(prediction != null) {
            Log.d(TAG, "prediction size: " + prediction.size());
//            Log.d(TAG, "prediction1: " + prediction.get(0, 0)[0]);
//            Log.d(TAG, "prediction2: " + prediction.get(0, 0)[1]);
//            Log.d(TAG, "prediction3: " + prediction.get(1, 0)[0]);
        }else{
            Log.d(TAG, "predictionIsNull");
        }
        if(prediction.get(0,0) != null && prediction.get(1,0) != null) {
            PointF predictPt = new PointF((float) prediction.get(0, 0)[0], (float) prediction.get(1, 0)[0]);
            targetPoint = predictPt;
            targetPoint.x = targetPoint.x / videoWidth;
            targetPoint.y = targetPoint.y / videoHeight;

            Log.d(TAG, "prediction point: " + predictPt);
        }
    }

    public boolean isTargetInVision() {
        targetPoint = getTargetPoint(testingActivity.yuv);
        if(targetPoint != null) {
            return true;
        }
        return false;
    }

    //-------------------------Get Video Source-------------------------

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
                temp.convertTo(temp2, CV_32F);

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
