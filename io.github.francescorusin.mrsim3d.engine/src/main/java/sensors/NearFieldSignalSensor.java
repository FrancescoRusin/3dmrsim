package sensors;

import bodies.AbstractBody;
import bodies.Voxel;
import engine.Ode4jEngine;
import sensors.Sensor;

import java.util.Arrays;

public class NearFieldSignalSensor implements Sensor {
    private final AbstractBody emitter;
    public final int channel;
    private final double[] readings;
    public NearFieldSignalSensor(AbstractBody emitter, int channel) {
        this.emitter = emitter;
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
        if (engine.t() % 1 < 1d / 60d) {
            //TODO TEST
            //System.out.println(Arrays.stream(readingsCopy).boxed().toList());
        }
        resetReadings();
        return readingsCopy;
    }
    @Override
    public int outputSize() {
        return 12;
    }
}
