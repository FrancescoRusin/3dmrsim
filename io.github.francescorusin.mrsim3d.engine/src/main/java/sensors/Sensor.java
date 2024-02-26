package sensors;

import agents.EmbodiedAgent;
import engine.Ode4jEngine;

public interface Sensor {
    double[] sense(Ode4jEngine engine);
}
