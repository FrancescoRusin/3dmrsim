package bodies;

import actions.Action;
import engine.Ode4jEngine;

import java.util.List;

public interface SignalEmitter extends AbstractBody {
    double DEFAULT_COMM_LENGTH = 1.5;
    List<Action> emitSignals(Ode4jEngine engine, double length, int channel, double[] values);
}
