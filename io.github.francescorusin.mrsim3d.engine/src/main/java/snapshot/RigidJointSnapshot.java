package snapshot;

import geometry.Vector3D;
import viewer.Viewer;

import java.awt.*;

public record RigidJointSnapshot(Vector3D point1, Vector3D point2) implements JointSnapshot {
    static final Color RIGID_JOINT_COLOR = Color.BLACK;

    @Override
    public double restLength() {
        return point1.vectorDistance(point2).norm();
    }

    @Override
    public void draw(Viewer viewer) {
        viewer.drawLine(point1(), point2(), RIGID_JOINT_COLOR);
    }
}
