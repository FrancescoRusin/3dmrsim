package actions;

import bodies.SignalEmitter;
import engine.Ode4jEngine;
import geometry.Vector3D;

public class EmitSignal implements Action {
    final SignalEmitter emitter;
    final Vector3D direction;
    final double length;
    final int channel;
    final double value;

    public EmitSignal(SignalEmitter emitter, Vector3D direction, double length, int channel, double value) {
        this.emitter = emitter;
        this.direction = direction;
        this.length = length;
        this.channel = channel;
        this.value = value;
    }

    @Override
    public void execute(Ode4jEngine engine) {
        engine.emitSignal(emitter, direction, length, channel, value);
    }
}
