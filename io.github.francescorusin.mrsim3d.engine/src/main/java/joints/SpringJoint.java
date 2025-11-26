package joints;

import engine.Ode4jEngine;
import geometry.Vector3D;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DDoubleBallJoint;
import snapshot.JointSnapshot;
import viewer.Viewer;

import java.awt.*;

public record SpringJoint(int id, DDoubleBallJoint joint) implements Joint {
    public record SoftJointSnapshot(Vector3D point1, Vector3D point2, double restLength) implements JointSnapshot {
        static final Color SOFT_JOINT_COLOR = Color.BLUE;

        @Override
        public void draw(Viewer viewer) {
            viewer.drawLine(point1, point2, SOFT_JOINT_COLOR);
        }
    }

    @Override
    public JointSnapshot snapshot(Ode4jEngine engine, Ode4jEngine.Mode mode) {
        DVector3 placeholder1 = new DVector3();
        joint.getAnchor1(placeholder1);
        DVector3 placeholder2 = new DVector3();
        joint.getAnchor2(placeholder2);
        return new SoftJointSnapshot(
                new Vector3D(placeholder1.get0(), placeholder1.get1(), placeholder1.get2()),
                new Vector3D(placeholder2.get0(), placeholder2.get1(), placeholder2.get2()),
                joint.getDistance()
        );
    }
}
