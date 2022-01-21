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

    private TargetDetect targetDetect;
    private Thread targetDetectionThread;
    boolean targetInVision = false;
    GimbalRotateTask gimbalRotateTask;
    Timer timerGimbalRotateTask;

    public VisualLanding() {
//        targetDetect = new TargetDetect();
    }

    public void startVisualLanding() {

        startGimbalTask(GimbalTaskMode.ADJUST);

        if(checkPrecisionLanding()){
            Thread visualLandingFlightControlThread = new Thread(new VisualLandingFlightControl());
            visualLandingFlightControlThread.start();
        }else {
            Log.d(TAG, "Start visual landing failed, because the target is not in vision.");
        }
    }

    private boolean checkPrecisionLanding() {

        if(targetDetect.isTargetInVision()) {
            return true;
        }else {
            VisualLandingFlightControl visualLandingFlightControl =
                    new VisualLandingFlightControl();
            visualLandingFlightControl.sendFlightUpCommand();

            if(targetDetect.isTargetInVision()) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void detectionCallback(boolean targetDetected) {
        targetInVision = targetDetected;
    }

    private void startGimbalTask(GimbalTaskMode gimbalTaskMode) {
        if(timerGimbalRotateTask != null) {
            try{
                TimeUnit.SECONDS.sleep(30);
            }catch (Exception e) {
                Log.d(TAG, "Gimbal rotation task running.");
            }
        }
        gimbalRotateTask = new GimbalRotateTask(gimbalTaskMode);
        timerGimbalRotateTask = new Timer();
        timerGimbalRotateTask.schedule(gimbalRotateTask, 0, 100);
    }
}
