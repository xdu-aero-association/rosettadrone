package sq.rogue.rosettadrone.autolanding;

import android.util.Log;

import androidx.annotation.NonNull;

import java.util.Timer;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.gimbal.GimbalState;
import dji.common.util.CommonCallbacks;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;
import sq.rogue.rosettadrone.RDApplication;
import sq.rogue.rosettadrone.settings.Tools;

public class VisualLanding implements DetectionCallback{

    private static final String TAG = "Visual landing";

    private FlightController flightController;
    private TargetDetect targetDetect;
    private Thread targetDetectionThread;
    boolean targetInVision = false;
    int gimbalAngleCnt = 0;
    GimbalRotateTask gimbalRotateTask;
    Timer timerGimbalRotateTask;

    public VisualLanding() {
        targetDetect = new TargetDetect();
    }

    public void startVisualLanding() {
        if(checkPrecisionLanding()){

            Thread visualLandingFlightControlThread = new Thread(new VisualLandingFlightControl());
            visualLandingFlightControlThread.start();

            //!!??

            //land
            flightController.startLanding(new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    if(djiError == null) {
                        Log.d(TAG, "Start landing failed, error: " + djiError.getDescription());
                    }
                }
            });

            flightController.setStateCallback(new FlightControllerState.Callback() {
                @Override
                public void onUpdate(@NonNull FlightControllerState flightControllerState) {
                    if (flightControllerState.isLandingConfirmationNeeded()) {
                        flightController.confirmLanding(new CommonCallbacks.CompletionCallback() {
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
        }
    }

    private boolean checkPrecisionLanding() {
        targetDetectionThread = new Thread(targetDetect);
        targetDetectionThread.start();

        gimbalRotateTask = new GimbalRotateTask(GimbalTaskMode.CHECK);
        timerGimbalRotateTask = new Timer();
        timerGimbalRotateTask.schedule(gimbalRotateTask, 0, 100);

        while(gimbalAngleCnt != 2 && !targetInVision) {

            RDApplication.getProductInstance().getGimbal().setStateCallback(new GimbalState.Callback() {
                @Override
                public void onUpdate(@NonNull GimbalState gimbalState) {
                    if(gimbalState.isPitchAtStop()) {
                        gimbalAngleCnt++;
                    }
                }
            });

            targetDetect.isTargetInVision(this);

            if(targetInVision) {
                //adjust finished, break
                break;
            }
        }

        if(targetInVision) {
            //stop target detection
            try{
                targetDetectionThread.interrupt();
            } catch (Exception e) {
                Log.d(TAG, "Error occurs while trying to end the target detection.");
            }

            //stop gimbal task
            if(timerGimbalRotateTask != null) {
                timerGimbalRotateTask.cancel();
                timerGimbalRotateTask.purge();
                timerGimbalRotateTask = null;
            }
            if(gimbalRotateTask != null) {
                gimbalRotateTask.cancel();
                gimbalRotateTask = null;
            }
        }else {
            Log.d(TAG, "Start visual landing failed, error: the target is not in vision.");
        }

        return targetInVision;
    }

    @Override
    public void detectionCallback(boolean targetDetected) {
        targetInVision = targetDetected;
    }
}
