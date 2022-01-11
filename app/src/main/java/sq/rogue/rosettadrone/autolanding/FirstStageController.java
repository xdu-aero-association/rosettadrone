package sq.rogue.rosettadrone.autolanding;

import android.app.Activity;
import android.graphics.PointF;

import androidx.annotation.Nullable;

import org.greenrobot.eventbus.EventBus;
import org.greenrobot.eventbus.Subscribe;
import org.greenrobot.eventbus.ThreadMode;

import dji.common.error.DJIError;
import dji.common.gimbal.GimbalState;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.mission.tapfly.TapFlyExecutionState;
import dji.common.mission.tapfly.TapFlyMission;
import dji.common.mission.tapfly.TapFlyMissionState;
import dji.common.mission.tapfly.TapFlyMode;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseProduct;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.mission.tapfly.TapFlyMissionEvent;
import dji.sdk.mission.tapfly.TapFlyMissionOperator;
import dji.sdk.mission.tapfly.TapFlyMissionOperatorListener;
import dji.sdk.sdkmanager.DJISDKManager;
import sq.rogue.rosettadrone.RDApplication;
import sq.rogue.rosettadrone.settings.Tools;

public class FirstStageController implements Runnable{

    public Activity mActivity;

    private BaseProduct product;
    private TapFlyMission tapFlyMission;
    private TapFlyMissionOperator tapFlyMissionOperator;
    private Gimbal gimbal;
    private float gimbalYawAngle;
    private static final float MAX_YAW_ANGLE = 90f;
    private static final float GIMBAL_ROTATE_DURATION = 2;

    private Runnable targetDetection;
    private Thread targetDetectionThread;

    public FirstStageController(Activity activity) {
        product = RDApplication.getProductInstance();
        gimbal = product.getGimbal();
        if(product == null || gimbal == null){
            Tools.showToast(mActivity, "Check the connection.");
        }
        EventBus.getDefault().register(this);
        mActivity = activity;
    }

    @Override
    public void run() {
        EventBus.getDefault().register(this);       //register for ThreadEvent

        targetDetection = new TargetDetecter();
        targetDetectionThread = new Thread(targetDetection, "targetDetectThread");
        targetDetectionThread.start();
    }

    //-------------------------Flight Control-------------------------
    @Subscribe(threadMode = ThreadMode.BACKGROUND)
    public void rotateGimbal(TargetPointResultEvent targetPointResultEvent) {
          //gimbal state update
        gimbal.setStateCallback(new GimbalState.Callback() {
            @Override
            public void onUpdate(GimbalState gimbalState) {
                gimbalYawAngle = gimbalState.getYawRelativeToAircraftHeading();
            }
        });


        if(targetPointResultEvent.isTargetInVision) {
            //post the thread event to start the TapFlyMission
            EventBus.getDefault().post(new ThreadEvent(true, targetPointResultEvent.targetPoint));
        } else {
            if(gimbalYawAngle == MAX_YAW_ANGLE) {
                Tools.showToast(mActivity, "The target is not in the vision, precision landing failed.");
                EventBus.getDefault().post(new ThreadEvent(true, ""));
            } else {
                rotateGimbal();
            }
        }
    }

    private void rotateGimbal() {
        Rotation.Builder builder = new Rotation.Builder().mode(RotationMode.ABSOLUTE_ANGLE).time(GIMBAL_ROTATE_DURATION);
        //???param
        builder.pitch(0);
        builder.yaw(0);
        builder.roll(0);

        gimbal.rotate(builder.build(), new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {

            }
        });
    }



    //-------------------------TapFlyMission-------------------------
    @Subscribe(threadMode =  ThreadMode.POSTING)
    private void startTapFlyMission(ThreadEvent threadEvent) {
        if(threadEvent.startTapFlyMission) {
            startTapFly(threadEvent.targetPoint);
        }

//        if(threadEvent.isMaxAngle) {
            try{
                targetDetectionThread.interrupt();
            } catch (Exception exception) {

            }
//        }
    }

    private void initTapFlyMission() {
        tapFlyMissionOperator = DJISDKManager.getInstance().getMissionControl().getTapFlyMissionOperator();
        tapFlyMission = new TapFlyMission();
        tapFlyMission.isHorizontalObstacleAvoidanceEnabled = true;
        tapFlyMission.tapFlyMode = TapFlyMode.FORWARD;

        tapFlyMissionOperator.addListener(new TapFlyMissionOperatorListener() {
            @Override
            public void onUpdate(@Nullable TapFlyMissionEvent aggregation) {
                TapFlyExecutionState executionState = aggregation.getExecutionState();
                if (executionState != null){

                }

                TapFlyExecutionState progressState = aggregation.getExecutionState();

                if (progressState != null) {

                }

                TapFlyMissionState missionState = aggregation.getCurrentState();
                if (!((missionState == TapFlyMissionState.EXECUTING) || (missionState == TapFlyMissionState.EXECUTION_PAUSED)
                        || (missionState == TapFlyMissionState.EXECUTION_RESETTING))){
                }else
                {

                }
            }
        });
    }

    private void startTapFly(PointF targetPoint) {
        initTapFlyMission();
        tapFlyMission.target = targetPoint;
        tapFlyMissionOperator.startMission(tapFlyMission, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {

            }
        });
    }

}
