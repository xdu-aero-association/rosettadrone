package sq.rogue.rosettadrone.autolanding;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import android.app.Activity;
import android.graphics.PointF;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import org.greenrobot.eventbus.EventBus;

import java.util.Timer;
import java.util.TimerTask;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.util.CommonCallbacks;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.products.Aircraft;
import sq.rogue.rosettadrone.R;
import sq.rogue.rosettadrone.RDApplication;

public class FlightControlActivity extends Activity implements View.OnClickListener{

    private static final String TAG = "FCTest";

    private Button enableBtn;
    private Button disableBtn;
    private Button takeoffBtn;
    private Button landBtn;
    private Button land2Btn;
    private EditText rollET;
    private EditText yawET;
    private EditText verticalET;
    private Button sendDataBtn;
    private Button fullStartBtn;

    private FlightController flightController;
    private Timer timerFlightDataTask;
    private FlightControlDataTask flightControlDataTask;
    private FlightControlData flightControlData;

    private float roll = 0;
    private float yaw = 0;
    private float vertical = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_flight_control);
        initUI();
        flightController = ((Aircraft) RDApplication.getProductInstance()).getFlightController();

    }

    private void initUI() {
        enableBtn = findViewById(R.id.enableBtn);
        disableBtn = findViewById(R.id.disableBtn);
        takeoffBtn = findViewById(R.id.takeoffBtn);
        landBtn = findViewById(R.id.landBtn);
        land2Btn = findViewById(R.id.land2Btn);
        rollET = findViewById(R.id.rollET);
        yawET = findViewById(R.id.yawET);
        verticalET = findViewById(R.id.verticalET);
        sendDataBtn = findViewById(R.id.sendDataBtn);
        fullStartBtn = findViewById(R.id.fullStartBtn);

        enableBtn.setOnClickListener(this);
        disableBtn.setOnClickListener(this);
        takeoffBtn.setOnClickListener(this);
        landBtn.setOnClickListener(this);
        land2Btn.setOnClickListener(this);
        sendDataBtn.setOnClickListener(this);
        fullStartBtn.setOnClickListener(this);
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.enableBtn: {
                initFlightControl();
                break;
            }
            case R.id.disableBtn: {
                endSecondFlightControl();
                break;
            }
            case R.id.takeoffBtn: {
                ((Aircraft)RDApplication.getProductInstance()).getFlightController()
                        .startTakeoff(new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onResult(DJIError djiError) {
                        if(djiError != null) {
                            Log.d(TAG, "tak-off error: " + djiError.getDescription());
                        }
                    }
                });
                break;
            }
            case R.id.landBtn: {
                ((Aircraft)RDApplication.getProductInstance()).getFlightController()
                        .startLanding(new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {
                                if(djiError != null) {
                                    Log.d(TAG, "landing error: " + djiError.getDescription());
                                }
                            }
                        });
                break;
            }
            case R.id.land2Btn: {
                ((Aircraft)RDApplication.getProductInstance()).getFlightController().startLanding(new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onResult(DJIError djiError) {

                    }
                });

                ((Aircraft)RDApplication.getProductInstance()).getFlightController().setStateCallback(new FlightControllerState.Callback() {
                    @Override
                    public void onUpdate(@NonNull FlightControllerState flightControllerState) {
                        if (flightControllerState.isLandingConfirmationNeeded()) {
                            ((Aircraft)RDApplication.getProductInstance()).getFlightController().confirmLanding(new CommonCallbacks.CompletionCallback() {
                                @Override
                                public void onResult(DJIError djiError) {

                                }
                            });
                        }
                    }
                });
                break;
            }
            case R.id.sendDataBtn: {
                if(timerFlightDataTask == null) {
                    flightControlData = setFlightControlData();
                    timerFlightDataTask = new Timer();
                    timerFlightDataTask.schedule(new FlightControlDataTask(), 1000, 200);
                } else {
                    flightControlData = setFlightControlData();
                }
                break;
            }
            case R.id.fullStartBtn: {
                Thread FCTest = new Thread(new SecondStageController(true));
            }
        }
    }

    private void initFlightControl() {
        if(flightController != null) {
            flightController.setVirtualStickModeEnabled(true, new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    if(djiError != null) {
                        if(djiError != null) {
                            Log.d(TAG, "Can't start virtual stick control with error: " + djiError.getDescription());
                        }
                    }
                }
            });

            //set the control mode
            flightController.setRollPitchControlMode(RollPitchControlMode.ANGLE);
            flightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
            flightController.setYawControlMode(YawControlMode.ANGLE);
            flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);
        } else {
            Log.d(TAG, "FlightController is null");
        }

    }

    public void endSecondFlightControl() {

        //flight control task cancel
        flightControlDataTask.cancel();
        timerFlightDataTask.cancel();
        timerFlightDataTask.purge();
        timerFlightDataTask = null;
        flightControlDataTask = null;

        //eventbus unregister
        EventBus.getDefault().unregister(this);

        //disable virtual stick control
        flightController.setVirtualStickModeEnabled(false, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {

            }
        });
    }

    public FlightControlData setFlightControlData() {
        if(rollET.getText().toString() != null) {
            roll = Float.parseFloat(rollET.getText().toString());
        }
        if(yawET.getText().toString() != null){
            yaw = Float.parseFloat(yawET.getText().toString());
        }
        if(verticalET.getText().toString() != null) {
            vertical = Float.parseFloat(verticalET.getText().toString());
        }

        return new FlightControlData(yaw, roll, 0, vertical);
    }

    private class FlightControlDataTask extends TimerTask {
        @Override
        public void run() {
            ((Aircraft)RDApplication.getProductInstance()).getFlightController().
                    sendVirtualStickFlightControlData(flightControlData, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if(djiError != null) {
                                Log.d(TAG, "Send flight control data error: " + djiError.getDescription());
                            }
                        }
                    });
        }
    }
}