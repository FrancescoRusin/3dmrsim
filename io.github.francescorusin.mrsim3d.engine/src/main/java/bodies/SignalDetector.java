package bodies;

import engine.Ode4jEngine;
import org.ode4j.ode.DRay;

public interface SignalDetector {
    void readSignal(Ode4jEngine engine, DRay signal);
}
