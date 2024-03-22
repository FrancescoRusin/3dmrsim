package sensors;

import bodies.AbstractBody;

public class ContactSensor extends InteractiveSensor {
    private final AbstractBody body;
    public ContactSensor(AbstractBody body) {
        this.body = body;
        this.currentState = new double[1];
    }
    public void detectContact() {
        currentState[0] = 1;
    }
    @Override
    void resetReadings() {
        currentState[0] = -1;
    }

    @Override
    public int outputSize() {
        return 1;
    }
}
