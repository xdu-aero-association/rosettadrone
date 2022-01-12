package sq.rogue.rosettadrone.autolanding;

import android.app.Activity;

import org.greenrobot.eventbus.Subscribe;
import org.greenrobot.eventbus.ThreadMode;

import java.util.Timer;

import dji.common.error.DJIError;
import dji.common.util.CommonCallbacks;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;
import sq.rogue.rosettadrone.RDApplication;
import sq.rogue.rosettadrone.settings.Tools;

//p:
//video data read in
//flight control param
//thread: event bus thread mode control
//activity

public class PrecisionLandingController {

    private FirstStageController firstStageController;
    private TargetDetecter targetDetecter;
    private Aircraft aircraft;
    private FlightController flightController;

    Runnable firstStageControllerRunnable;
    Thread firstStageControllerThread;
    SecondStageController secondStageControllerTask;
    Timer secondStageController;

    public Activity mActivity;      //???

    public PrecisionLandingController() {
        aircraft = (Aircraft) RDApplication.getProductInstance();
        if(aircraft == null) {
            Tools.showToast(mActivity, "Check the connection.");
        }
//        EventBus.getDefault().register(this);
    }

    public void firstStageOfPrecisionLanding() {
        //head to the target area roughly
        //start the 2 threads: gimbal rotate & image process →
        //once the target is in vision, interrupt the current thread to start the TapFlyMission
        firstStageControllerRunnable = new FirstStageController(mActivity);
        firstStageControllerThread = new Thread(firstStageControllerRunnable, "firstStageControllerThread");
        firstStageControllerThread.start();
    }

    public void secondStageOfPrecisionLanding() {
        //start 2 threads: control through virtual stick(Time Task) & image process
        flightController = aircraft.getFlightController();
        flightController.setVirtualStickModeEnabled(true, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {

            }
        });
        secondStageControllerTask = new SecondStageController();
        secondStageController = new Timer();
        secondStageController.schedule(secondStageControllerTask, 1000, 200);
    }

    public void thirdStageOfPrecisionLanding() {
        //landing
                    //???
        flightController.startLanding(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {

            }
        });
    }

    public boolean checkBeforeStartPrecisionLanding() {
        //height
        //check first stage


        //check second stage
        if(!flightController.isVirtualStickControlModeAvailable()) {
            return false;
        }

        //check third stage

        return true;
    }

    public void startPrecisionLanding() {
        if(checkBeforeStartPrecisionLanding()){
            firstStageOfPrecisionLanding();
            secondStageOfPrecisionLanding();
            thirdStageOfPrecisionLanding();
        }
    }

    @Subscribe(threadMode = ThreadMode.POSTING)
    public void threadInterrupt(ThreadEvent threadEvent) {

        if(threadEvent.end) {

        }
    }

}
