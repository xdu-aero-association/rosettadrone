package sq.rogue.rosettadrone.autolanding;

public class PIDParamChangeEvent {
    int mode;
    float a, b, c;

    PIDParamChangeEvent(float a, float b, float c, int mode) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.mode = mode;
    }
}
