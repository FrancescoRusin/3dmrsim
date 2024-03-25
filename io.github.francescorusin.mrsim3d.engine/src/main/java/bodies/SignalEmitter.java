package bodies;

import actions.Action;
import engine.Ode4jEngine;

import java.util.List;

public interface SignalEmitter extends AbstractBody {
    List<Action> emitSignals(Ode4jEngine engine, int channel, double[] values);
}
