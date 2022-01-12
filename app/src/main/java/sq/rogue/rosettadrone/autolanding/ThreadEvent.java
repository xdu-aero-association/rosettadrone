package sq.rogue.rosettadrone.autolanding;

import android.graphics.PointF;

public class ThreadEvent {
    boolean end = false;
    boolean startTapFlyMission = false;
    boolean isMaxAngle = false;
    PointF targetPoint;
    String threadName;


    public ThreadEvent(boolean end, String threadName) {
        this.end = end;
        this.threadName = threadName;
    }

    public ThreadEvent(boolean startTapFlyMission, PointF targetPoint) {
        this.startTapFlyMission = startTapFlyMission;
        this.targetPoint = targetPoint;
    }

    public ThreadEvent(boolean isMaxAngle) {
        this.isMaxAngle = isMaxAngle;
    }
}
