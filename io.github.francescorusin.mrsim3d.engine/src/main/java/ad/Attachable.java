package ad;

import bodies.Body;
import engine.SimulationObject;

import java.util.List;

public interface Attachable extends SimulationObject {
    List<List<Body>> attachPossibilities();
}
