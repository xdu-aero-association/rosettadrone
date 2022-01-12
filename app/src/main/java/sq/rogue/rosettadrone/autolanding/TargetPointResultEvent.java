package sq.rogue.rosettadrone.autolanding;

import android.graphics.PointF;

public class TargetPointResultEvent {
    protected boolean isTargetInVision;
    protected PointF targetPoint;

    public TargetPointResultEvent(PointF targetPoint, boolean isTargetInVision) {
        this.isTargetInVision = isTargetInVision;
        this.targetPoint = targetPoint;
    }
}
