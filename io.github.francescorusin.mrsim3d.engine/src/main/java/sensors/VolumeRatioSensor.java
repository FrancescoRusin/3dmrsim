package sensors;

import bodies.SoftBody;
import bodies.Voxel;
import engine.Ode4jEngine;

public class VolumeRatioSensor implements Sensor {
    SoftBody body;
    public VolumeRatioSensor(SoftBody body) {
        this.body = body;
    }
    @Override
    public double[] sense(Ode4jEngine engine) {
        return new double[]{body.currentVolume() / body.restVolume()};
    }

    @Override
    public int outputSize() {
        return 1;
    }
}
