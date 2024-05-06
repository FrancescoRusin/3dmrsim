package snapshot;

import geometry.Vector3D;

import java.util.List;

public interface AgentSnapshot extends ObjectSnapshot {
    List<ObjectSnapshot> components();

    @Override
    default Vector3D position() {
        List<ObjectSnapshot> components = components();
        return components.stream().map(ObjectSnapshot::position).reduce(Vector3D::sum).orElse(new Vector3D()).times(1d / components.size());
    }
}
