package sq.rogue.rosettadrone.autolanding;

import android.util.Log;
import android.util.TimeUtils;

import androidx.annotation.NonNull;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.gimbal.GimbalState;
import dji.common.util.CommonCallbacks;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;
import sq.rogue.rosettadrone.RDApplication;
import sq.rogue.rosettadrone.settings.Tools;

public class VisualLanding implements DetectionCallback{

    private static final String TAG = "Visual landing";

    protected DJICodecManager djiCodecManager;
    private TargetDetect targetDetect;
    private VisualLandingFlightControl visualLandingFlightControl;
    boolean targetInVision = false;
    GimbalRotateTask gimbalRotateTask;
    Timer timerGimbalRotateTask;

    public VisualLanding(DJICodecManager djiCodecManager) {
        this.djiCodecManager = djiCodecManager;
        targetDetect = new TargetDetect(djiCodecManager);
        visualLandingFlightControl = new VisualLandingFlightControl(djiCodecManager);
    }

    public void startVisualLanding() {

        startGimbalTask(GimbalTaskMode.ADJUST);

        if(checkPrecisionLanding()){
            Thread visualLandingFlightControlThread = new Thread(visualLandingFlightControl);
            visualLandingFlightControlThread.start();
        }else {
            Log.d(TAG, "Start visual landing failed, because the target is not in vision.");
        }

        endGimbalTask();
    }

    private boolean checkPrecisionLanding() {

        if(targetDetect.isTargetInVision()) {
            return true;
        }else {
            if(((Aircraft)RDApplication.getProductInstance()).getFlightController()
                    .getState().getAircraftLocation().getAltitude() < 8) {
                visualLandingFlightControl.sendFlightUpCommand();
            }

            if(targetDetect.isTargetInVision()) {
                visualLandingFlightControl.endCheckFlight();
                return true;
            }
        }
        visualLandingFlightControl.endCheckFlight();
        return false;
    }

    @Override
    public void detectionCallback(boolean targetDetected) {
        targetInVision = targetDetected;
    }

    private void startGimbalTask(GimbalTaskMode gimbalTaskMode) {
        gimbalRotateTask = new GimbalRotateTask(gimbalTaskMode);
        timerGimbalRotateTask = new Timer();
        timerGimbalRotateTask.schedule(gimbalRotateTask, 0, 100);
    }

    private void endGimbalTask() {
        if(gimbalRotateTask != null) {
            gimbalRotateTask.cancel();
        }
        if(timerGimbalRotateTask != null) {
            timerGimbalRotateTask.cancel();
            timerGimbalRotateTask.purge();
        }
        timerGimbalRotateTask = null;
        gimbalRotateTask = null;
    }
}
