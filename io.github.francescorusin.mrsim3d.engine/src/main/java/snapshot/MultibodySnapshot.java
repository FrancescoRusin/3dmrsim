package snapshot;

import geometry.Vector3D;

import java.util.List;

public interface MultibodySnapshot extends BodySnapshot {
    List<BodySnapshot> bodyParts();

    List<JointSnapshot> internalJoints();
    @Override
    default Vector3D position() {
        return bodyParts().stream().map(b -> b.position().times(b.mass())).reduce(Vector3D::sum).orElseThrow().times(1d / mass());
    }

    @Override
    default Vector3D velocity() {
        return bodyParts().stream().map(b -> b.velocity().times(b.mass())).reduce(Vector3D::sum).orElseThrow().times(1d / mass());
    }

    @Override
    default double mass() {
        return bodyParts().stream().mapToDouble(BodySnapshot::mass).sum();
    }
}
