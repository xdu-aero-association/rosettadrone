package sq.rogue.rosettadrone.autolanding;

import android.graphics.PointF;

import androidx.annotation.NonNull;

import java.util.TimerTask;

import dji.common.error.DJIError;
import dji.common.gimbal.GimbalState;
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

    GimbalRotateTask(GimbalTaskMode gimbalTaskMode) {
        //first rotate before the precision landing start
        this.gimbalTaskMode = gimbalTaskMode;
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
            pitch = -90;
        } else if(gimbalTaskMode == GimbalTaskMode.CHECK) {
            rotationMode = RotationMode.SPEED;
            pitch = -40;
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