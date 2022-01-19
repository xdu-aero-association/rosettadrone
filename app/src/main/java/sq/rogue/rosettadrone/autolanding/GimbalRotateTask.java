package sq.rogue.rosettadrone.autolanding;

import android.graphics.PointF;

import java.util.TimerTask;

import dji.common.error.DJIError;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.util.CommonCallbacks;
import sq.rogue.rosettadrone.RDApplication;

public class GimbalRotateTask extends TimerTask {
    float pitch = -40;
    PointF preTargetPoint = null;
    PointF curTargetPoint = null;
    float ka = 5f;
    float kb = 8f;
    GimbalTaskMode gimbalTaskMode;
    RotationMode rotationMode;

    GimbalRotateTask(PointF inTargetPoint) {
        super();
        preTargetPoint = curTargetPoint;
        curTargetPoint = inTargetPoint;
    }

    GimbalRotateTask(GimbalTaskMode gimbalTaskMode) {
        //first rotate before the precision landing start
        this.gimbalTaskMode = gimbalTaskMode;
    }



    public void setTargetPoint(PointF inTargetPoint) {
        preTargetPoint = curTargetPoint;
        curTargetPoint = inTargetPoint;
    }

    private float setPitch() {
        if(preTargetPoint != null) {
            if( (Math.abs(preTargetPoint.y - 0.5f) < Math.abs(curTargetPoint.y) - 0.5f) ) {
                pitch = ka * (curTargetPoint.y - 0.5f) + kb * (curTargetPoint.y - preTargetPoint.y);
            }
        }
        return pitch;
    }

    @Override
    public void run() {
        if(gimbalTaskMode == GimbalTaskMode.ADJUST) {
            rotationMode = RotationMode.ABSOLUTE_ANGLE;
        } else if(gimbalTaskMode == GimbalTaskMode.CHECK) {
            rotationMode = RotationMode.SPEED;
        } else if(gimbalTaskMode == GimbalTaskMode.POINT) {
            pitch = setPitch();
        }

        RDApplication.getProductInstance().getGimbal().
                rotate(new Rotation.Builder().pitch(pitch)
                        .mode(rotationMode)
                        .yaw(Rotation.NO_ROTATION)
                        .roll(Rotation.NO_ROTATION)
                        .time(0)
                        .build(), new CommonCallbacks.CompletionCallback() {

                    @Override
                    public void onResult(DJIError error) {

                    }
                });
    }
}

