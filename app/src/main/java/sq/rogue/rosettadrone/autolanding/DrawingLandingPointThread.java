package sq.rogue.rosettadrone.autolanding;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PointF;
import android.graphics.PorterDuff;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import org.greenrobot.eventbus.EventBus;
import org.greenrobot.eventbus.Subscribe;
import org.greenrobot.eventbus.ThreadMode;

public class DrawingLandingPointThread extends Thread{

//    private SurfaceView drawingSurface;
    private SurfaceHolder drawingHolder;
    private Paint paint;
    private int height;
    private int width;
    public PointF targetPoint = null;

    public boolean drawingControl = true;

    DrawingLandingPointThread(SurfaceView drawingSurface) {
//        this.drawingSurface = drawingSurface;
        drawingHolder = drawingSurface.getHolder();
        paint.setColor(Color.BLUE);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(5f);
        EventBus.getDefault().register(this);
    }

    @Subscribe(threadMode = ThreadMode.MAIN)
    public void setTargetPoint(TargetPointResultEvent targetPointResultEvent) {
        targetPoint = targetPointResultEvent.targetPoint;
    }

    @Override
    public void run() {
        while(drawingControl) {

            Canvas canvas = drawingHolder.lockCanvas();

            if (targetPoint != null) {
                canvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);
                canvas.drawCircle(0, 0, 5f, paint);
            }

            try{
                Thread.sleep(6000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
