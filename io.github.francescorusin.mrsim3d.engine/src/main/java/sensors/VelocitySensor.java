package sensors;

import bodies.AbstractBody;
import engine.Ode4jEngine;
import geometry.Vector3D;

public class VelocitySensor implements Sensor {
    private final AbstractBody body;

    public VelocitySensor(AbstractBody body) {
        this.body = body;
    }

    @Override
    public double[] sense(Ode4jEngine engine) {
        Vector3D vectorVelocity = body.velocity(engine.t()).reverseRotate(body.angle(engine.t()));
        return new double[]{vectorVelocity.x(), vectorVelocity.y(), vectorVelocity.z()};
    }

    @Override
    public int outputSize() {
        return 3;
    }
}
