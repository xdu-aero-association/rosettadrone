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
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.util.CommonCallbacks;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.products.Aircraft;
import sq.rogue.rosettadrone.RDApplication;

public class SecondStageController implements Runnable{

    private static final String TAG = "SecondStage";

    private boolean test = false;

    private FlightController flightController;
    private PointF targetPoint = null;
    private Gimbal gimbal;

    private FlightControlData flightControlData;
    private Timer timerFlightDataTask;
    private FlightControlDataTask flightControlDataTask;
    private Timer timerGimbalRotation;
    private GimbalRotateTask gimbalRotateTask;
    private Timer timerTargetDetection;
    private TargetDetect targetDetectionTask;

    public SecondStageController(boolean testStateOn) {
        test = testStateOn;
    }

    public SecondStageController() {

    }

    @Override
    public void run() {
        //enable virtual stick control
        initFlightControl();

        //gimbal rotate
        gimbalRotateTask = new GimbalRotateTask(RotationMode.ABSOLUTE_ANGLE);
        timerGimbalRotation = new Timer();
        timerGimbalRotation.schedule(gimbalRotateTask, 0, 100);

        //gimbal rotation task cancel
        if(timerGimbalRotation != null) {
            timerGimbalRotation.cancel();
            timerGimbalRotation.purge();
            timerFlightDataTask = null;
        }
        if(gimbalRotateTask != null) {
            gimbalRotateTask.cancel();
            flightControlDataTask = null;
        }

        //build target detection task
        targetDetectionTask = new TargetDetect();
        timerTargetDetection = new Timer();
        timerTargetDetection.schedule(targetDetectionTask, 0, 100);

        //EventBus register
        EventBus.getDefault().register(this);

        //build flight control task
        timerFlightDataTask = new Timer();
        timerFlightDataTask.schedule(new FlightControlDataTask(), 1000, 200);

        //build gimbal rotate task
        gimbalRotateTask = new GimbalRotateTask(targetPoint);
        timerGimbalRotation = new Timer();
        timerGimbalRotation.schedule(gimbalRotateTask, 0, 100);
    }

    private void initFlightControl() {
        flightController = ((Aircraft) RDApplication.getProductInstance()).getFlightController();
        if(flightController != null) {
            flightController.setVirtualStickModeEnabled(true, new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    Log.d(TAG, "Can't start virtual stick control with error: " + djiError.getDescription());
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
        if(targetDetectionTask != null) {
            targetDetectionTask.cancel();
        }
        if(timerTargetDetection != null) {
            timerTargetDetection.cancel();
            timerTargetDetection.purge();
        }
        targetDetectionTask = null;
        timerTargetDetection = null;

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
    }

    public void setFlightControlData(PointF targetPoint) {
        //reset the flight control data according to the detection result
        float rollAngleCo = 0.1f;
        float pitchAngleCo = 0.1f;
        float verticalThrottleCo = 0.1f;

        float altitude = ((Aircraft)RDApplication.getProductInstance()).getFlightController()
                .getState().getAircraftLocation().getAltitude();

        if(altitude < 30) {

        } if(altitude < 10) {

        }

        flightControlData = new FlightControlData(
                altitude*pitchAngleCo,
                (targetPoint.x - 0.5f)*rollAngleCo,
                0,
                (targetPoint.y = 0.5f)*verticalThrottleCo );
    }

    private class FlightControlDataTask extends TimerTask {
        //send flight control data
        @Override
        public void run() {
            ((Aircraft)RDApplication.getProductInstance()).getFlightController().
                    sendVirtualStickFlightControlData(flightControlData, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {

                        }
                    });
        }
    }

    //-------------------------Gimbal Control-------------------------
    private static class GimbalRotateTask extends TimerTask {
        float pitch = 45;
        PointF preTargetPoint = null;
        PointF curTargetPoint = null;
        float ka = 0.1f;
        float kb = 0.1f;
        RotationMode rotationMode;

        GimbalRotateTask(PointF targetPoint) {
            super();
            preTargetPoint = curTargetPoint;
            curTargetPoint = targetPoint;
        }

        GimbalRotateTask(RotationMode rotationMode) {
            //first rotate before the precision landing start
            this.rotationMode = rotationMode;
        }

        private float setPitch() {
            if(preTargetPoint != null) {
                if( (Math.abs(preTargetPoint.y - 0.5f) < Math.abs(curTargetPoint.y) - 0.5f) ) {
                    pitch = ka * (curTargetPoint.y - 0.5f) + kb * (curTargetPoint.y - preTargetPoint.y);
                }
            }
            return pitch;
        }

        @Override
        public void run() {
            if(rotationMode == RotationMode.ABSOLUTE_ANGLE) {
                RDApplication.getProductInstance().getGimbal().
                        rotate(new Rotation.Builder().pitch(pitch)
                                .mode(RotationMode.ABSOLUTE_ANGLE)
                                .yaw(Rotation.NO_ROTATION)
                                .roll(Rotation.NO_ROTATION)
                                .time(0)
                                .build(), new CommonCallbacks.CompletionCallback() {

                            @Override
                            public void onResult(DJIError error) {

                            }
                        });
            } else {
                RDApplication.getProductInstance().getGimbal().
                        rotate(new Rotation.Builder().pitch(setPitch())
                                .mode(RotationMode.RELATIVE_ANGLE)
                                .yaw(Rotation.NO_ROTATION)
                                .roll(Rotation.NO_ROTATION)
                                .time(0)
                                .build(), new CommonCallbacks.CompletionCallback() {

                            @Override
                            public void onResult(DJIError error) {

                            }
                        });
            }
        }
    }
}
