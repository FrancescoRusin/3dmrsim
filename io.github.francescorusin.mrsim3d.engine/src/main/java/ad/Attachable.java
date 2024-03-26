package ad;

import bodies.Body;
import engine.Ode4jEngine;
import engine.SimulationObject;

import java.util.List;
import java.util.Map;
import java.util.Set;

public interface Attachable extends SimulationObject {
    List<List<Body>> attachPossibilities();
    Map<Body, Set<Body>> attachedBodies();
    boolean checkAttachment(Body body);
    default boolean checkAttachment(Attachable attachable) {
        for (Body body : attachable.bodyParts()) {
            if (checkAttachment(body)) {
                return true;
            }
        }
        return false;
    }
}
