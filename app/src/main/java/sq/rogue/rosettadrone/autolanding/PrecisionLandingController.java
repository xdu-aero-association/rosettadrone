package sq.rogue.rosettadrone.autolanding;

import androidx.annotation.NonNull;

import org.greenrobot.eventbus.Subscribe;
import org.greenrobot.eventbus.ThreadMode;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.util.CommonCallbacks;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;
import sq.rogue.rosettadrone.RDApplication;

public class PrecisionLandingController {

    private FlightController flightController;

    public PrecisionLandingController() {
        flightController = ((Aircraft) RDApplication.getProductInstance()).getFlightController();
    }

    public void startPrecisionLanding() {
        if(checkPrecisionLanding()){
//            firstStageOfPrecisionLanding();
            secondStageOfPrecisionLanding();
        }
    }

//    private void firstStageOfPrecisionLanding() {
//        //going to be deleted
//
//        //head to the target area roughly
//        //start the 2 threads: gimbal rotate & image process →
//        //once the target is in vision, interrupt the current thread to start the TapFlyMission
//        Thread firstStageControllerThread = new Thread(new FirstStageController(), "firstStageControllerThread");
//        firstStageControllerThread.start();
//    }

    private void secondStageOfPrecisionLanding() {
        //start 2 threads: control through virtual stick(Time Task) & image process
        Thread secondStageControllerThread = new Thread(new SecondStageController());
        secondStageControllerThread.start();
    }

    private void thirdStageOfPrecisionLanding() {
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

    private boolean checkPrecisionLanding() {
        if(!flightController.isVirtualStickControlModeAvailable()) {
            return false;
        }
        if(flightController == null) {
            return false;
        }
        //height
        return true;
    }


    @Subscribe(threadMode = ThreadMode.POSTING)
    public void threadInterrupt(ThreadEvent threadEvent) {
        thirdStageOfPrecisionLanding();
    }
}
