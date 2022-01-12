package sq.rogue.rosettadrone.autolanding;

import android.graphics.PointF;

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
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.util.CommonCallbacks;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.products.Aircraft;
import sq.rogue.rosettadrone.RDApplication;

public class SecondStageController extends TimerTask {

    private static final float GIMBAL_ROTATE_DURATION = 2;
    private FlightController flightController;
    private Aircraft aircraft;
    private FlightControllerState aFlightControllerState;
    private PointF targetPoint;
    private Gimbal gimbal;

    //--control param--
    private float pitch;
    private float roll;
    private float yaw;
    private float verticalThrottle;
    private FlightControlData flightControlData;

    @Override
    public void run() {
        EventBus.getDefault().register(this);

        aircraft = (Aircraft) RDApplication.getProductInstance();
        flightController = aircraft.getFlightController();
        gimbal = aircraft.getGimbal();


        //get aircraft location
        flightController.setStateCallback(new FlightControllerState.Callback() {
            @Override
            public void onUpdate(FlightControllerState flightControllerState) {
                aFlightControllerState = flightControllerState;
            }
        });


        //set the control mode
        flightController.setRollPitchControlMode(RollPitchControlMode.ANGLE);
        flightController.setVerticalControlMode(VerticalControlMode.POSITION);
        flightController.setYawControlMode(YawControlMode.ANGLE);
        flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);

        //
//        yaw =
        flightControlData = setFlightControlData();


        //build control task
        flightController.sendVirtualStickFlightControlData(flightControlData, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {

            }
        });

    }

    @Subscribe(threadMode = ThreadMode.MAIN)
    public void flightControl(TargetPointResultEvent targetPointResultEvent) {


        //get point
        targetPoint = targetPointResultEvent.targetPoint;

        //start sending virtual stick control task
    }

    public FlightControlData setFlightControlData() {
        //
        return new FlightControlData(pitch, roll, yaw, verticalThrottle);
    }

}

