package bodies;

import sensors.Sensor;

import java.util.List;

public interface SensingBody extends AbstractBody {
    List<Sensor> sensors();
}
