package sensors;

import bodies.SignalDetector;
import bodies.Voxel;

public class NearFieldSignalSensor extends InteractiveSensor {
    public final int channel;
    public NearFieldSignalSensor(SignalDetector body, int channel) {
        this.channel = channel;
        currentState = new double[body.nOfSides()];
    }

    public void readSignal(double value, int side) {
        currentState[side] += value;
    }
}
