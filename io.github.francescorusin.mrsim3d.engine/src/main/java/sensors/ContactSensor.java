package sensors;

public class ContactSensor extends StatefulSensor {
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
