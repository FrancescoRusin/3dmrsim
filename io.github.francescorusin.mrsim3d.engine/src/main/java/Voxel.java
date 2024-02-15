import geometry.BoundingBox;
import geometry.Vector3D;
import org.ode4j.ode.DBallJoint;

import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

public class Voxel extends MultiBody {
    public enum Vertex {
        V000, V001, V010, V011, V100, V101, V110, V111
    }

    public enum JointOption {
        INTERAL, SIDES, EDGES
    }
    EnumSet<JointOption> jointOptions;
    Map<Vertex, RigidBody> rigidBodies;
    List<DBallJoint> joints;
    List<RigidBody> ulteriorBodies;
    double sideLength;
    double mass;
    double friction;
    double springConstant;
    double dampingConstant;
    double rigidMassLengthRatio;
    double[] areaRatio;
    public Voxel(double sideLength,
                 double mass,
                 double friction,
                 double springConstant,
                 double dampingConstant,
                 double rigidMassLengthRatio,
                 double minAreaRatio,
                 double maxAreaRatio,
                 EnumSet<JointOption> jointOptions) {
        this.sideLength = sideLength;
        this.mass = mass;
        this.friction = friction;
        this.springConstant = springConstant;
        this.dampingConstant = dampingConstant;
        this.rigidMassLengthRatio = rigidMassLengthRatio;
        this.areaRatio = new double[]{minAreaRatio > 0 ? minAreaRatio : 0, maxAreaRatio};
        this.jointOptions = jointOptions;
    }
    @Override
    public List<Body> getBodyParts() {
        return Stream.concat(rigidBodies.values().stream(), ulteriorBodies.stream()).map(b -> (Body) b).toList();
    }

    @Override
    public void assemble(Engine engine, Vector3D position) {
    }
}
