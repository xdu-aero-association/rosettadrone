package sq.rogue.rosettadrone.autolanding;

import android.graphics.PointF;
import android.view.View;

import androidx.annotation.Nullable;

import dji.common.error.DJIError;
import dji.common.mission.tapfly.TapFlyMission;
import dji.common.mission.tapfly.TapFlyMode;
import dji.common.util.CommonCallbacks;
import dji.sdk.mission.tapfly.TapFlyMissionEvent;
import dji.sdk.mission.tapfly.TapFlyMissionOperator;
import dji.sdk.mission.tapfly.TapFlyMissionOperatorListener;
import dji.sdk.sdkmanager.DJISDKManager;

public class TapFlyControl implements Runnable{
    private TapFlyMission tapFlyMission;

    private TapFlyMissionOperator getTapFlyOperator() {
        return DJISDKManager.getInstance().getMissionControl().getTapFlyMissionOperator();
    }

    @Override
    public void run() {
        getTapFlyOperator().addListener(new TapFlyMissionOperatorListener() {
            @Override
            public void onUpdate(@Nullable TapFlyMissionEvent tapFlyMissionEvent) {

            }
        });
        getTapFlyOperator().startMission(tapFlyMission, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {

            }
        });
    }

    private void initTapFlyMission() {
        tapFlyMission = new TapFlyMission();
        getTapFlyOperator().setAutoFlightSpeed(1, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {

            }
        });
        tapFlyMission.isHorizontalObstacleAvoidanceEnabled = true;
        tapFlyMission.tapFlyMode = TapFlyMode.FORWARD;
//        tapFlyMission.target = getTapFlyPoint(new View());
    }

    private PointF getTapFlyPoint(View iv) {
        if (iv == null) return null;
        View parent = (View)iv.getParent();
        float centerX = iv.getLeft() + iv.getX()  + ((float)iv.getWidth()) / 2;
        float centerY = iv.getTop() + iv.getY() + ((float)iv.getHeight()) / 2;
        centerX = centerX < 0 ? 0 : centerX;
        centerX = centerX > parent.getWidth() ? parent.getWidth() : centerX;
        centerY = centerY < 0 ? 0 : centerY;
        centerY = centerY > parent.getHeight() ? parent.getHeight() : centerY;

        return new PointF(centerX / parent.getWidth(), centerY / parent.getHeight());
    }

    private void showPointByTapFlyPoint(){

    };



}
