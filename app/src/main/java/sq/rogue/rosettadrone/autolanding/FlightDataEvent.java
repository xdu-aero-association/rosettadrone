package sq.rogue.rosettadrone.autolanding;

public class FlightDataEvent {
    public float yaw;
    public float pitch;
    public float roll;
    public float throttle;

    public FlightDataEvent(float yaw, float pitch, float roll, float throttle) {
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
        this.throttle = throttle;
    }
}
