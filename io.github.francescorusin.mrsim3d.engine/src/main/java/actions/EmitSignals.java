package actions;

import bodies.Voxel;
import engine.Ode4jEngine;
import geometry.Vector3D;

import java.util.Arrays;
import java.util.EnumMap;

public class EmitSignals implements Action {
    final Voxel emitter;
    final double length;
    final EnumMap<Voxel.Side, Double> values;

    public EmitSignals(Voxel emitter, double length, EnumMap<Voxel.Side, Double> values) {
        this.emitter = emitter;
        this.length = length;
        this.values = values;
    }

    @Override
    public void execute(Ode4jEngine engine) {
        Vector3D emitterAngle = emitter.angle(engine.t());
        Vector3D emitterPosition = emitter.position(engine.t());
        double tick = emitter.bodyCenterToBodyCenterLength() / 2;
        for (Voxel.Side side : Voxel.Side.values()) {
            String name = side.name();
            Vector3D direction = new Vector3D(
                    emitterPosition.x() + tick * (name.charAt(1) == 0 ? -1 : 1),
                    emitterPosition.x() + tick * (name.charAt(2) == 0 ? -1 : 1),
                    emitterPosition.x() + tick * (name.charAt(3) == 0 ? -1 : 1)
            ).rotate(emitterAngle);
            engine.emitSignal(emitter, direction, length, values.get(side));
        }
    }
}
