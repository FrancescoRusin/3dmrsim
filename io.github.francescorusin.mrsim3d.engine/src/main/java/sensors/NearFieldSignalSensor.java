package sensors;

import bodies.Voxel;
import engine.Ode4jEngine;

import java.util.Arrays;

public class NearFieldSignalSensor implements Sensor {
    private final Voxel voxel;
    public final int channel;
    private final double[] readings;
    public NearFieldSignalSensor(Voxel voxel, int channel) {
        this.voxel = voxel;
        this.channel = channel;
        readings = new double[12];
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
        return 12;
    }
}
