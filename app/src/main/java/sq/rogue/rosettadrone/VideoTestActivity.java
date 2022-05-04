package sq.rogue.rosettadrone;

import androidx.appcompat.app.AppCompatActivity;

import android.graphics.SurfaceTexture;
import android.os.Bundle;
import android.view.TextureView;
import android.view.View;

import dji.sdk.codec.DJICodecManager;

public class VideoTestActivity extends AppCompatActivity implements TextureView.SurfaceTextureListener{

    private TextureView ttv;
    private DJICodecManager djiCodecManager;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_video_test);

        ttv = findViewById(R.id.livestream_preview_ttv_test);
        ttv.setSurfaceTextureListener(this);
        ttv.setVisibility(View.VISIBLE);
    }


    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surfaceTexture, int i, int i1) {
        if(djiCodecManager == null) {
            djiCodecManager = new DJICodecManager(this, surfaceTexture, i, i1);
        }
    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surfaceTexture, int i, int i1) {

    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surfaceTexture) {
        if(djiCodecManager != null) {
            djiCodecManager.cleanSurface();
            djiCodecManager = null;
        }
        return false;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surfaceTexture) {

    }
}