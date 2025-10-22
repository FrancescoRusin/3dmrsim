package joints;

import engine.Ode4jEngine;
import geometry.Vector3D;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DFixedJoint;
import snapshot.JointSnapshot;
import viewer.Viewer;

import java.awt.*;

public record FixedJoint(int id, DFixedJoint joint) implements Joint {
    public record FixedJointSnapshot(Vector3D point1, Vector3D point2) implements JointSnapshot {
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
    @Override
    public JointSnapshot snapshot(Ode4jEngine engine) {
        DVector3C placeholder1 = joint.getBody(0).getPosition();
        DVector3C placeholder2 = joint.getBody(0).getPosition();
        Vector3D pos1 = new Vector3D(placeholder1.get0(), placeholder1.get1(), placeholder1.get2());
        Vector3D pos2 = new Vector3D(placeholder2.get0(), placeholder2.get1(), placeholder2.get2());
        return new FixedJointSnapshot(pos1, pos2);
    }
}
