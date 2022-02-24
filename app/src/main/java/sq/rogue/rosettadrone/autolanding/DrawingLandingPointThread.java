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

    @Subscribe(threadMode = ThreadMode.MAIN)
    public void setTargetPoint(TargetPointResultEvent targetPointResultEvent) {
        targetPoint = targetPointResultEvent.targetPoint;
    }

    @Override
    public void run() {

        Log.d(TAG, "DrawingPointStart");

        EventBus.getDefault().register(this);

        while(drawingControl) {

            if (targetPoint != null) {
                Canvas canvas = drawingHolder.lockCanvas();
                canvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);
                canvas.drawCircle(targetPoint.x*width, targetPoint.y*height, 50f, paint);
                Log.d(TAG, "DrawThePoint"+" "+targetPoint.x+" "+targetPoint.y);
                drawingHolder.unlockCanvasAndPost(canvas);
            }

            try{
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
