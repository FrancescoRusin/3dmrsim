package ad;

import bodies.Body;
import bodies.SimulationObject;
import geometry.Vector3D;

import java.util.List;
import java.util.Map;
import java.util.Set;

public interface Attachable extends SimulationObject {
    //TODO MAYBE SWITCH LIST<BODY> WITH AN ARRAY OR A RECORD
    List<List<Body>> attachPossibilities();

    Map<List<Body>, Vector3D> attachPossibilitiesPositions(double t);

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