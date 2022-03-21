package sq.rogue.rosettadrone.autolanding;

import android.util.Log;

import java.util.Timer;

import dji.sdk.codec.DJICodecManager;
import dji.sdk.products.Aircraft;
import sq.rogue.rosettadrone.RDApplication;

//pid param opposite
//coordinate system relative
//exit → time, is
//straight?  coordinate
//gimbal

public class VisualLanding {

    private static final String TAG = "Visual landing";

    protected DJICodecManager djiCodecManager;
    private TargetDetect targetDetect;
    protected VisualLandingFlightControl visualLandingFlightControl;
    GimbalRotateTask gimbalRotateTask;
    Timer timerGimbalRotateTask;

    public VisualLanding(DJICodecManager djiCodecManager) {
        this.djiCodecManager = djiCodecManager;
        targetDetect = new TargetDetect(djiCodecManager);
        visualLandingFlightControl = new VisualLandingFlightControl(djiCodecManager);
    }

    public VisualLanding() {

    }

    public void startVisualLanding() {

        startGimbalTask(GimbalTaskMode.ADJUST);

        if(checkVisualLanding()){
            Thread visualLandingFlightControlThread = new Thread(visualLandingFlightControl);
            visualLandingFlightControlThread.start();
            Log.d(TAG, "visualLandingFlightControl start");
        }else {
            Log.d(TAG, "Start visual landing failed, because the target is not in vision.");
        }
    }

    private boolean checkVisualLanding() {

        Log.d(TAG, "check visual landing");
        if(targetDetect.isTargetInVision()) {
            return true;
        }else {
            if(((Aircraft)RDApplication.getProductInstance()).getFlightController()
                    .getState().getAircraftLocation().getAltitude() < 8) {
                visualLandingFlightControl.sendFlightUpCommand();
                visualLandingFlightControl.endCheckFlight();
                Log.d(TAG, "sendFlightUpCommand");
            }

            if(targetDetect.isTargetInVision()) {
                visualLandingFlightControl.endCheckFlight();
                return true;
            }
        }
        return false;
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
