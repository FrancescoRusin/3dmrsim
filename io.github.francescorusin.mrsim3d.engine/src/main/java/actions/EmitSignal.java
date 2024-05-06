package actions;

import bodies.SignalEmitter;
import engine.Ode4jEngine;
import geometry.Vector3D;
import viewer.Viewer;

public class EmitSignal implements Action {
    final SignalEmitter emitter;
    final Vector3D direction;
    final int channel;
    final double value;

    public EmitSignal(SignalEmitter emitter, Vector3D direction, int channel, double value) {
        this.emitter = emitter;
        this.direction = direction;
        this.channel = channel;
        this.value = value;
    }

    @Override
    public void execute(Ode4jEngine engine) {
        engine.emitSignal(emitter, direction, channel, value);
    }

    @Override
    public void draw(Viewer viewer) {
        //TODO IMPLEMENT
    }
}
