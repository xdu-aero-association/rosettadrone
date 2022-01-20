package sq.rogue.rosettadrone.autolanding;

import android.graphics.PointF;
import android.util.Log;

import org.greenrobot.eventbus.EventBus;
import org.greenrobot.eventbus.Subscribe;
import org.greenrobot.eventbus.ThreadMode;

import java.util.Timer;
import java.util.TimerTask;

import dji.common.error.DJIError;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.util.CommonCallbacks;
import dji.lidar_map.my_point_3d.Point3D;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.products.Aircraft;
import sq.rogue.rosettadrone.RDApplication;
import sq.rogue.rosettadrone.settings.Tools;

public class VisualLandingFlightControl implements Runnable{

    private static final String TAG = "Visual FC control";

    private boolean testMode = false;
    private Point3D target;

    private FlightController flightController;
    private PointF targetPoint = null;
    private Gimbal gimbal;

    private FlightControlData flightControlData;
    private Timer timerFlightDataTask;
    private FlightControlDataTask flightControlDataTask;
    private Timer timerGimbalRotation;
    private GimbalRotateTask gimbalRotateTask;
    private Thread targetDetectionThread;

    VisualLandingFlightControl(boolean testMode, Point3D target) {
        this.testMode = testMode;
        this.target = target;
    }

    VisualLandingFlightControl() {

    }

    @Override
    public void run() {
        //enable virtual stick control
        initFlightControl();

        //gimbal rotate
        gimbalRotateTask = new GimbalRotateTask(GimbalTaskMode.ADJUST);
        timerGimbalRotation = new Timer();
        timerGimbalRotation.schedule(gimbalRotateTask, 0, 100);

        //gimbal rotation task cancel
        if(timerGimbalRotation != null) {
            timerGimbalRotation.cancel();
            timerGimbalRotation.purge();
            timerGimbalRotation = null;
        }
        if(gimbalRotateTask != null) {
            gimbalRotateTask.cancel();
            gimbalRotateTask = null;
        }

        //build target detection task
        if(!testMode) {
            targetDetectionThread = new Thread(new TargetDetect());
            targetDetectionThread.start();
        }

        //EventBus register
        EventBus.getDefault().register(this);

        //build flight control task
        timerFlightDataTask = new Timer();
        timerFlightDataTask.schedule(new FlightControlDataTask(), 1000, 200);
    }

    private void initFlightControl() {
        flightController = ((Aircraft) RDApplication.getProductInstance()).getFlightController();
        if(flightController != null) {
            flightController.setVirtualStickModeEnabled(true, new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    if(djiError != null) {
                        Log.d(TAG, "Can't start virtual stick control with error: " + djiError.getDescription());
                    }
                }
            });

            flightControlData = new FlightControlData(0, 0, 0, 0);      //relative control mode init

            //set the control mode
            flightController.setRollPitchControlMode(RollPitchControlMode.ANGLE);
            flightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
            flightController.setYawControlMode(YawControlMode.ANGLE);
            flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);

            gimbal = RDApplication.getProductInstance().getGimbal();

        } else {
            Log.d(TAG, "FlightController is null");
        }
    }

    public void endSecondFlightControl() {
        //gimbal rotation task cancel
        if(gimbalRotateTask != null) {
            gimbalRotateTask.cancel();
        }
        if(timerGimbalRotation != null) {
            timerGimbalRotation.cancel();
            timerGimbalRotation.purge();
        }
        timerGimbalRotation = null;
        gimbalRotateTask = null;

        //flight control task cancel
        if(flightControlDataTask != null) {
            flightControlDataTask.cancel();
        }
        if(timerFlightDataTask != null) {
            timerFlightDataTask.cancel();
            timerFlightDataTask.purge();
        }
        timerFlightDataTask = null;
        flightControlDataTask = null;

        //target detection task cancel
        if(!testMode) {
            try {
                targetDetectionThread.interrupt();
            } catch (Exception e) {
                Log.d(TAG, "Error occurs while trying to end the target detection.");
            }
        }

        //eventbus unregister
        EventBus.getDefault().unregister(this);

        //disable virtual stick control
        flightController.setVirtualStickModeEnabled(false, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {

            }
        });
    }

    @Subscribe(threadMode = ThreadMode.MAIN)
    public void end(ThreadEvent threadEvent) {
        endSecondFlightControl();
    }

    //-------------------------Flight Control-------------------------
    @Subscribe(threadMode = ThreadMode.MAIN)
    public void flightControl(TargetPointResultEvent targetPointResultEvent) {
        //receive the target detection result
        targetPoint = targetPointResultEvent.targetPoint;
        setFlightControlData(targetPoint);
        gimbalRotateTask.setTargetPoint(targetPoint);
    }


    float errorXI = 0;
    float errorYI = 0;
    float altitudeI = 0;
    float errorXPre = 0;
    float errorYPre = 0;
    float altitudePre = 0;
    public void setFlightControlData(PointF targetPoint) {

        float errorXCur = targetPoint.x - 0.5f;
        float errorYCur = targetPoint.y - 0.5f;
        errorXI += errorXCur;
        errorYI += errorYCur;

        //--------------------roll--------------------
        //specific
        float rollP = 0.1f;
        float rollI = 0.1f;
        float rollD = 0.1f;
        //roll angle range in (-0.4, 0.4)
        float rollAngle = rollP*errorXCur+rollI*errorXI+rollD*(errorXCur-errorXPre);

        //--------------------vertical--------------------
        //specific
        float verticalThrottleP = 0.1f;
        float verticalThrottleI = 0.1f;
        float verticalThrottleD = 0.1f;
        float altitudeCur = ((Aircraft)RDApplication.getProductInstance()).getFlightController()
                .getState().getAircraftLocation().getAltitude();
        altitudeI += altitudeCur;
        //roll angle range in (-5, 5)
        float verticalThrottle = verticalThrottleP*altitudeCur+verticalThrottleI*altitudeI+verticalThrottleD*(altitudeCur-altitudePre);

        //--------------------pitch--------------------
        //specific
        float pitchP = 0.1f;
        float pitchI = 0.1f;
        float pitchD = 0.1f;
        //pitch angle: (0.00001, 0.001)
        float pitchAngle = pitchP*errorXCur+pitchI*errorXI+pitchD*(errorXCur-errorXPre);

        flightControlData = new FlightControlData(pitchAngle, rollAngle, 0, verticalThrottle);

        errorXPre = errorXCur;
        errorYPre = errorYCur;
        altitudePre = altitudeCur;
    }

    public void setFlightControlData2() {
        //for test

        //build the center point
        float longitudeCur = (float) ((Aircraft)RDApplication.getProductInstance()).getFlightController()
                .getState().getAircraftLocation().getLongitude();
        float latitudeCur = (float) ((Aircraft)RDApplication.getProductInstance()).getFlightController()
                .getState().getAircraftLocation().getLatitude();
        float altitudeCur = ((Aircraft)RDApplication.getProductInstance()).getFlightController()
                .getState().getAircraftLocation().getAltitude();

        Point3D flightPoint = new Point3D(longitudeCur, latitudeCur, altitudeCur);

        Log.d(TAG, "flight point" +flightPoint);
        Log.d(TAG, "target point" +target);

        float errorXCur = flightPoint.x - target.x;
        float errorYCur = flightPoint.y - target.y;
        errorXI += errorXCur;
        errorYI += errorYCur;

        //--------------------roll--------------------
        //specific
        float rollP = 0.1f;
        float rollI = 0.1f;
        float rollD = 0.1f;
        //roll angle range in (-0.4, 0.4)
        float rollAngle = rollP*errorXCur+rollI*errorXI+rollD*(errorXCur-errorXPre);

        //--------------------vertical--------------------
        //specific
        float verticalThrottleP = 0.1f;
        float verticalThrottleI = 0.1f;
        float verticalThrottleD = 0.1f;

        altitudeI += altitudeCur;
        //roll angle range in (-5, 5)
        float verticalThrottle = verticalThrottleP*altitudeCur+verticalThrottleI*altitudeI+verticalThrottleD*(altitudeCur-altitudePre);

        //--------------------pitch--------------------
        //specific
        float pitchP = 0.1f;
        float pitchI = 0.1f;
        float pitchD = 0.1f;
        //pitch angle: (0.00001, 0.001)
        float pitchAngle = pitchP*errorXCur+pitchI*errorXI+pitchD*(errorXCur-errorXPre);

        flightControlData = new FlightControlData(pitchAngle, rollAngle, 0, verticalThrottle);

        errorXPre = errorXCur;
        errorYPre = errorYCur;
        altitudePre = altitudeCur;
    }

    private class FlightControlDataTask extends TimerTask {
        //send flight control data
        @Override
        public void run() {
            if(testMode) {
                setFlightControlData2();
            }
            ((Aircraft) RDApplication.getProductInstance()).getFlightController().
                    sendVirtualStickFlightControlData(flightControlData, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {

                        }
                    });
        }
    }
}
