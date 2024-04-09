package sensors;

import engine.Ode4jEngine;

import java.util.Arrays;

public abstract class StatefulSensor implements Sensor {
    // sensors that do not sense by themselves and instead read some cached value updated by external actions;
    // as different sensors potentially update differently, no standard state update function is implemented
    protected double[] currentState;
    void resetReadings() {
        Arrays.fill(currentState, 0d);
    }

    @Override
    public double[] sense(Ode4jEngine engine) {
        double[] readingsCopy = new double[currentState.length];
        for (int i = 0; i < currentState.length; ++i) {
            readingsCopy[i] = Math.max(-1d, Math.min(1d, currentState[i]));
        }
        resetReadings();
        return readingsCopy;
    }

    @Override
    public int outputSize() {
        return currentState.length;
    }
}
