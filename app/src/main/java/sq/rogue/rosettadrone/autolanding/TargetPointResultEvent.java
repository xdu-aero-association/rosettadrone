package sq.rogue.rosettadrone.autolanding;

import android.graphics.PointF;
import android.util.Log;

public class TargetPointResultEvent {
    protected PointF targetPoint = null;
    protected float radius = 0;

    public TargetPointResultEvent(PointF targetPoint, float radius) {
        this.targetPoint = targetPoint;
        this.radius = radius;
    }

    public TargetPointResultEvent(){

    }

    public void setPoint(PointF newPoint, float newRadius) {
        if(targetPoint == null){
            targetPoint = newPoint;
            return;
        }

        if(newPoint == null) {
            return;
        }else{
//            if(Math.abs(newPoint.x - targetPoint.x) > 0.15 || Math.abs(newPoint.y - targetPoint.y) > 0.15) {
//                return;
//            }else{
                targetPoint = newPoint;
                radius = newRadius;
//            }
        }
    }
}
