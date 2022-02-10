package sq.rogue.rosettadrone.autolanding;

import android.graphics.PointF;
import android.util.Log;

public class TargetPointResultEvent {
    protected boolean isTargetInVision;
    protected PointF targetPoint;

    public TargetPointResultEvent(PointF targetPoint, boolean isTargetInVision) {
        this.isTargetInVision = isTargetInVision;
        this.targetPoint = targetPoint;
    }
}
