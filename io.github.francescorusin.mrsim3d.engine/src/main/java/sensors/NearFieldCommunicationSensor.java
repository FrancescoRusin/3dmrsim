package sensors;

import bodies.SignalDetector;

public class NearFieldCommunicationSensor extends InteractiveSensor {
    public final int channel;
    public NearFieldCommunicationSensor(SignalDetector body, int channel) {
        this.channel = channel;
        currentState = new double[body.nOfSides()];
    }

    public void readSignal(double value, int side) {
        currentState[side] += value;
    }
}
