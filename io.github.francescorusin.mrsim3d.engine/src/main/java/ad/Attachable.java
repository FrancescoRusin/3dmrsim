package ad;

import bodies.Body;
import bodies.SimulationObject;

import java.util.List;

public interface Attachable extends SimulationObject {
    double DEFAULT_ATTACH_DISTANCE = .5;
    double DEFAULT_ATTRACT_DISTANCE = 2d;
    List<List<Body>> attachPossibilities();
}
