package sq.rogue.rosettadrone.autolanding;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PointF;
import android.graphics.PorterDuff;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import org.greenrobot.eventbus.EventBus;
import org.greenrobot.eventbus.Subscribe;
import org.greenrobot.eventbus.ThreadMode;

public class DrawingLandingPointThread extends Thread{

    private static final String TAG = "DrawingLandingPoint";

//    private SurfaceView drawingSurface;
    private SurfaceHolder drawingHolder;
    private Paint paint = new Paint();
    private int height;
    private int width;
    public PointF targetPoint = null;
    private float radius = 4f;

    public boolean drawingControl = true;

    DrawingLandingPointThread(SurfaceView drawingSurface,int height, int width) {
//        this.drawingSurface = drawingSurface;
        drawingHolder = drawingSurface.getHolder();

        this.height = height;
        this.width = width;

        paint.setColor(Color.BLUE);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(100f);
    }

    @Subscribe(sticky = true, threadMode = ThreadMode.ASYNC)
    public void setTargetPoint(TargetPointResultEvent targetPointResultEvent) {
        if(targetPointResultEvent.targetPoint != null) {
            targetPoint = targetPointResultEvent.targetPoint;
            this.radius = targetPointResultEvent.radius;
            Log.d(TAG, " thePointIs: " + targetPoint);
        }
    }

    @Override
    public void run() {

        EventBus.getDefault().register(this);
        Canvas canvas = new Canvas();

        while(drawingControl) {
            long time1 = System.currentTimeMillis();

            canvas = drawingHolder.lockCanvas();
            canvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);
            if(targetPoint != null) {
                canvas.drawCircle(targetPoint.x * width, targetPoint.y * height, 50f, paint);
                long time2 = System.currentTimeMillis();
                long duration = time2 - time1;
                Log.d(TAG, "DrawThePoint"+" "+targetPoint.x+" "+targetPoint.y
                        + "\n" + " ProcessingDuration: " + duration);
            }
            drawingHolder.unlockCanvasAndPost(canvas);

//            try{
//                Thread.sleep(50);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
        }
    }
}
