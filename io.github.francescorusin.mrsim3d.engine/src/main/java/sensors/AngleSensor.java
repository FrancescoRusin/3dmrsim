package sensors;

import agents.EmbodiedAgent;
import bodies.Body;
import engine.Ode4jEngine;

public class AngleSensor implements Sensor {
    Body body;
    @Override
    public double[] sense(Ode4jEngine engine) {
        return body.angle();
    }

    public int arity() {
        return 3;
    }
}
