package sq.rogue.rosettadrone.autolanding;

import androidx.annotation.NonNull;

import java.util.Timer;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.gimbal.GimbalState;
import dji.common.util.CommonCallbacks;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;
import sq.rogue.rosettadrone.RDApplication;

public class VisualLanding implements DetectionCallback{

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

                }
            });

            flightController.setStateCallback(new FlightControllerState.Callback() {
                @Override
                public void onUpdate(@NonNull FlightControllerState flightControllerState) {
                    if (flightControllerState.isLandingConfirmationNeeded()) {
                        flightController.confirmLanding(new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {

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
            //fly up
            //!!??
        }

        return targetInVision;
    }

    @Override
    public void detectionCallback(boolean targetDetected) {
        targetInVision = targetDetected;
    }
}
