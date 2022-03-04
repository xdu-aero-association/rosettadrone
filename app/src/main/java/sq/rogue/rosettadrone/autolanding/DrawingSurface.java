package sq.rogue.rosettadrone.autolanding;

import android.app.Activity;
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PixelFormat;
import android.graphics.PointF;
import android.graphics.PorterDuff;
import android.util.AttributeSet;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;

import org.greenrobot.eventbus.EventBus;
import org.greenrobot.eventbus.Subscribe;
import org.greenrobot.eventbus.ThreadMode;

import sq.rogue.rosettadrone.MainActivity;

public class DrawingSurface extends SurfaceView implements SurfaceHolder.Callback, Runnable{

    public boolean drawing = true;

    private SurfaceHolder drawingHolder;
    private Paint paint = new Paint();
    private int height;
    private int width;
    public PointF targetPoint = null;

    public DrawingSurface(Context context) {
        super(context);
    }

    public DrawingSurface(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    public DrawingSurface(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
    }

    public DrawingSurface(Context context, AttributeSet attrs, int defStyleAttr, int defStyleRes) {
        super(context, attrs, defStyleAttr, defStyleRes);
    }

    private void init() {
        this.getHolder().addCallback(this);
        this.setZOrderOnTop(true);
        this.getHolder().setFormat(PixelFormat.TRANSPARENT);
        this.setVisibility(View.VISIBLE);
    }

    @Override
    public void surfaceCreated(SurfaceHolder surfaceHolder) {

    }

    @Override
    public void surfaceChanged(SurfaceHolder surfaceHolder, int i, int i1, int i2) {

    }

    @Override
    public void surfaceDestroyed(SurfaceHolder surfaceHolder) {

    }

    @Override
    public void run() {

        EventBus.getDefault().register(this);

        while(drawing) {
            drawingView();

            try{
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private void drawingView() {
        if (targetPoint != null) {
            Canvas canvas = drawingHolder.lockCanvas();
            canvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);
            canvas.drawCircle(targetPoint.x*width, targetPoint.y*height, 50f, paint);
            drawingHolder.unlockCanvasAndPost(canvas);
        }
    }

    @Subscribe(sticky = true, threadMode = ThreadMode.MAIN)
    public void setTargetPoint(TargetPointResultEvent targetPointResultEvent) {
        targetPoint = targetPointResultEvent.targetPoint;
    }
}
