package sensors;

import bodies.AbstractBody;
import bodies.SignalDetector;
import bodies.Voxel;
import engine.Ode4jEngine;
import sensors.Sensor;

import java.util.Arrays;

public class NearFieldSignalSensor implements Sensor {
    private final SignalDetector body;
    public final int channel;
    private final double[] readings;
    public NearFieldSignalSensor(SignalDetector body, int channel) {
        this.body = body;
        this.channel = channel;
        readings = new double[body.nOfSides()];
    }
    public void resetReadings() {
        Arrays.fill(readings, 0d);
    }

    public void readSignal(double value, Voxel.Side side) {
        readings[side.ordinal()] += value;
    }

    @Override
    public double[] sense(Ode4jEngine engine) {
        double[] readingsCopy = new double[readings.length];
        for (int i = 0; i < readings.length; ++i) {
            readingsCopy[i] = Math.max(-1d, Math.min(1d, readings[i]));
        }
        resetReadings();
        return readingsCopy;
    }
    @Override
    public int outputSize() {
        return body.nOfSides();
    }
}
