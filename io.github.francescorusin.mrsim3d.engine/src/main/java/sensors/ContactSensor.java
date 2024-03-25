package sensors;

import bodies.AbstractBody;

public class ContactSensor extends InteractiveSensor {
    public ContactSensor() {
        this.currentState = new double[1];
    }
    public void detectContact() {
        currentState[0] = 1;
    }
    @Override
    void resetReadings() {
        currentState[0] = -1;
    }
}
