package sq.rogue.rosettadrone.autolanding;

import android.graphics.PointF;
import android.util.Log;

import androidx.annotation.NonNull;

import org.greenrobot.eventbus.EventBus;
import org.greenrobot.eventbus.Subscribe;
import org.greenrobot.eventbus.ThreadMode;

import java.util.Timer;
import java.util.TimerTask;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.util.CommonCallbacks;
import dji.lidar_map.my_point_3d.Point3D;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.products.Aircraft;
import sq.rogue.rosettadrone.RDApplication;

public class VisualLandingFlightControl implements Runnable{

    private static final String TAG = "Visual FC control";

    private boolean testMode = false;
    private Point3D target;

    private FlightController flightController;
    private PointF targetPoint = null;

    private FlightControlData flightControlData;
    private Timer timerFlightDataTask;
    private FlightControlDataTask flightControlDataTask;
    private Timer timerGimbalRotation;
    private GimbalRotateTask gimbalRotateTask;
    private Thread targetDetectionThread;
    private DJICodecManager djiCodecManager;

    VisualLandingFlightControl(boolean testMode, Point3D target) {
        this.testMode = testMode;
        this.target = target;
        Log.d(TAG, "Target point: " + target.x + target.y + target.z);
    }

    VisualLandingFlightControl(DJICodecManager djiCodecManager) {
        this.djiCodecManager = djiCodecManager;
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
        targetDetectionThread = new Thread(new TargetDetect(djiCodecManager));
        targetDetectionThread.start();

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
            if(testMode) {
                flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.GROUND);
            }else {
                flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);
            }

        } else {
            Log.d(TAG, "FlightController is null");
        }
    }

    public void endVisualLandingFlightControl() {
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

    public void endCheckFlight() {
        if(flightControlDataTask != null) {
            flightControlDataTask.cancel();
        }
        if(timerFlightDataTask != null) {
            timerFlightDataTask.cancel();
            timerFlightDataTask.purge();
        }
        timerFlightDataTask = null;
        flightControlDataTask = null;
    }

    public void sendFlightUpCommand() {
        initFlightControl();
        flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.GROUND);
        flightController.setVerticalControlMode(VerticalControlMode.POSITION);
        flightControlData = new FlightControlData(0, 0, 0, 8);
        timerFlightDataTask = new Timer();
        timerFlightDataTask.schedule(new FlightControlDataTask(), 0, 200);
    }

    @Subscribe(threadMode = ThreadMode.ASYNC)
    public void end(TargetAtCenterEvent targetAtCenterEvent) {
        endVisualLandingFlightControl();
        //land
        ((Aircraft)RDApplication.getProductInstance()).getFlightController().startLanding(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                if(djiError == null) {
                    Log.d(TAG, "Start landing failed, error: " + djiError.getDescription());
                }
            }
        });

        ((Aircraft)RDApplication.getProductInstance()).getFlightController().setStateCallback(new FlightControllerState.Callback() {
            @Override
            public void onUpdate(@NonNull FlightControllerState flightControllerState) {
                if (flightControllerState.isLandingConfirmationNeeded()) {
                    ((Aircraft)RDApplication.getProductInstance()).getFlightController().confirmLanding(new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if(djiError == null) {
                                Log.d(TAG, "Confirm landing failed, error: " + djiError.getDescription());
                            }
                        }
                    });
                }
            }
        });

        //eventbus unregister
        EventBus.getDefault().unregister(this);

        //disable virtual stick control
        flightController.setVirtualStickModeEnabled(false, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {

            }
        });
    }

    //-------------------------Flight Control-------------------------
    @Subscribe(threadMode = ThreadMode.ASYNC)
    public void flightControl(TargetPointResultEvent targetPointResultEvent) {
        //receive the target detection result
        targetPoint = targetPointResultEvent.targetPoint;
        if(testMode){
            setFlightControlData3(targetPoint);
        }else {
            setFlightControlData(targetPoint);
        }
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

        //--------------------pitch--------------------
        //specific
        float pitchP = -0.0001f;
        float pitchI = 0f;
        float pitchD = -0.01f;
        //pitch angle: (0.00001, 0.001)
        float pitchAngle = pitchP*errorXCur+pitchI*errorXI+pitchD*(errorXCur-errorXPre);

        //--------------------roll--------------------
        //specific
        float rollP = 0.1f;
        float rollI = 0.001f;
        float rollD = 0.1f;
        //roll angle range in (-0.4, 0.4)
        float rollAngle = rollP*errorXCur+rollI*errorXI+rollD*(errorXCur-errorXPre);

        //--------------------vertical--------------------
        //specific
        float verticalThrottleP = -0.1f;
        float verticalThrottleI = -0.0001f;
        float verticalThrottleD = 0.1f;
        float altitudeCur = ((Aircraft)RDApplication.getProductInstance()).getFlightController()
                .getState().getAircraftLocation().getAltitude();
        altitudeI += altitudeCur;
        //roll angle range in (-5, 5)
//        float verticalThrottle = verticalThrottleP*altitudeCur+verticalThrottleI*altitudeI+verticalThrottleD*(altitudeCur-altitudePre);
        float verticalThrottle = 0;

        flightControlData = new FlightControlData(pitchAngle, rollAngle, 0, verticalThrottle);

        errorXPre = errorXCur;
        errorYPre = errorYCur;
        altitudePre = altitudeCur;
    }

    public void setFlightControlData2() {
        //for test

        //build the center point
        float latitudeCur = (float) ((Aircraft)RDApplication.getProductInstance()).getFlightController()
                .getState().getAircraftLocation().getLatitude();

        float longitudeCur = (float) ((Aircraft)RDApplication.getProductInstance()).getFlightController()
                .getState().getAircraftLocation().getLongitude();

        float altitudeCur = ((Aircraft)RDApplication.getProductInstance()).getFlightController()
                .getState().getAircraftLocation().getAltitude();

        Point3D flightPoint = new Point3D(latitudeCur, longitudeCur, altitudeCur);

        float errorXCur = flightPoint.x - target.x;     //0.1
        float errorYCur = flightPoint.y - target.y;
        errorXI += errorXCur;
        errorYI += errorYCur;

        //--------------------pitch--------------------
        //specific
        float pitchP = -0.01f;
        float pitchI = 0.0000001f;
        float pitchD = 0.001f;
        //pitch angle: (0.00001, 0.001)
        float pitchAngle = pitchP*errorXCur+pitchI*errorXI+pitchD*(errorXCur-errorXPre);

        //--------------------roll--------------------
        //specific
        float rollP = 0.4f;
        float rollI = 0.0000001f;
        float rollD = 1f;
        //roll angle range in (-0.4, 0.4)
        float rollAngle = rollP*errorXCur+rollI*errorXI+rollD*(errorXCur-errorXPre);

        //--------------------vertical--------------------
        //specific
        float verticalThrottleP = -0.1f;
        float verticalThrottleI = 0;
        float verticalThrottleD = -0.1f;

        altitudeI += altitudeCur;
        //roll angle range in (-5, 5)
        float verticalThrottle = verticalThrottleP*altitudeCur+verticalThrottleI*altitudeI+verticalThrottleD*(altitudeCur-altitudePre);

        flightControlData = new FlightControlData(pitchAngle, rollAngle, 0, verticalThrottle);

        Log.d(TAG, "Flight control data: " + flightControlData.getPitch()+" "+flightControlData.getRoll()+" "+flightControlData.getVerticalThrottle());
        Log.d(TAG, "Flight body data: " + flightPoint.x+" "+flightPoint.y+" "+flightPoint.z);

        errorXPre = errorXCur;
        errorYPre = errorYCur;
        altitudePre = altitudeCur;
    }

    float pitchP = -0.0001f;
    float pitchI = 0f;
    float pitchD = -0.01f;

    float rollP = 0.1f;
    float rollI = 0.001f;
    float rollD = 0.1f;

    float verticalThrottleP = -0.1f;
    float verticalThrottleI = -0.0001f;
    float verticalThrottleD = 0.1f;

    @Subscribe(threadMode = ThreadMode.ASYNC)
    public void setPIDParam(PIDParamChangeEvent pidParamChangeEvent) {
        if(pidParamChangeEvent.mode == 1) {
            pitchP = pidParamChangeEvent.a;
            pitchI = pidParamChangeEvent.b;
            pitchD = pidParamChangeEvent.c;
        }else if(pidParamChangeEvent.mode == 2) {
            rollP = pidParamChangeEvent.a;
            rollI = pidParamChangeEvent.b;
            rollD = pidParamChangeEvent.c;
        }else if(pidParamChangeEvent.mode == 3) {
            verticalThrottleP = pidParamChangeEvent.a;
            verticalThrottleI = pidParamChangeEvent.b;
            verticalThrottleD = pidParamChangeEvent.c;
        }
    }

    public void setFlightControlData3(PointF targetPoint) {

        float errorXCur = targetPoint.x - 0.5f;
        float errorYCur = targetPoint.y - 0.5f;
        errorXI += errorXCur;
        errorYI += errorYCur;

        //--------------------pitch--------------------
        //specific

        //pitch angle: (0.00001, 0.001)
        float pitchAngle = pitchP*errorXCur+pitchI*errorXI+pitchD*(errorXCur-errorXPre);

        //--------------------roll--------------------
        //specific

        //roll angle range in (-0.4, 0.4)
        float rollAngle = rollP*errorXCur+rollI*errorXI+rollD*(errorXCur-errorXPre);

        //--------------------vertical--------------------
        //specific

        float altitudeCur = ((Aircraft)RDApplication.getProductInstance()).getFlightController()
                .getState().getAircraftLocation().getAltitude();
        altitudeI += altitudeCur;
        //roll angle range in (-5, 5)
        float verticalThrottle = verticalThrottleP*altitudeCur+verticalThrottleI*altitudeI+verticalThrottleD*(altitudeCur-altitudePre);

        flightControlData = new FlightControlData(pitchAngle, rollAngle, 0, verticalThrottle);

        errorXPre = errorXCur;
        errorYPre = errorYCur;
        altitudePre = altitudeCur;
    }

    private class FlightControlDataTask extends TimerTask {
        //send flight control data
        @Override
        public void run() {
            ((Aircraft) RDApplication.getProductInstance()).getFlightController().
                    sendVirtualStickFlightControlData(flightControlData, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if(djiError != null) {
                                endVisualLandingFlightControl();
                            }
                        }
                    });
        }
    }
}
