package bodies;

import engine.Ode4jEngine;
import geometry.Vector3D;
import org.ode4j.ode.DRay;

import java.util.List;

public interface SignalDetector extends SensingBody {
    List<Body> detectorBodies();
    int nOfSides();
    void readSignal(Ode4jEngine engine, DRay signal, Vector3D contactPosition);
}
