import geometry.BoundingBox;
import geometry.Vector3D;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.OdeHelper;

public final class Sphere extends RigidBody {
    DSphere sphere;
    Vector3D center;
    double radius;
    double mass;
    public Sphere(Vector3D center, double radius, double mass) {
        this.center = center;
        this.radius = radius;
        this.mass = mass;
        this.sphere = OdeHelper.createSphere(radius);
    }
    @Override
    public BoundingBox boundingBox() {

        return new BoundingBox(
                new Vector3D(center.x() - radius, center.y() - radius, center.z() - radius),
                new Vector3D(center.x() + radius, center.y() + radius, center.z() + radius));
    }

    @Override
    public double mass() {
        return mass;
    }

    @Override
    public void assemble(Engine engine, Vector3D position) {
        //TODO
    }
}
