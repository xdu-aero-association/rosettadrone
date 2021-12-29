package sq.rogue.rosettadrone.autolanding;

public class Event {
    private int mMsg;
    public Event(int msg) {
        mMsg = msg;
    }
    public int getMsg(){
        return mMsg;
    }
}
