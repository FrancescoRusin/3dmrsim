package bodies;

import engine.Ode4jEngine;
import sensors.Sensor;

import java.util.List;

public interface SensingBody extends AbstractBody {
    List<Sensor> sensors();
    double[] getSensorReadings(Ode4jEngine engine);
}
