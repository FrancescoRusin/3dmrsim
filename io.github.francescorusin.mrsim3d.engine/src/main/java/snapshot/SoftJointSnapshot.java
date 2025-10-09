package snapshot;

import geometry.Vector3D;
import viewer.Viewer;

import java.awt.*;

public record SoftJointSnapshot(Vector3D point1, Vector3D point2, double restLength) implements JointSnapshot {
    static final Color SOFT_JOINT_COLOR = Color.BLUE;

    @Override
    public void draw(Viewer viewer) {
        viewer.drawLine(point1, point2, SOFT_JOINT_COLOR);
    }
}
