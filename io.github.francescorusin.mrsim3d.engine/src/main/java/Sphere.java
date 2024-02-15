import geometry.BoundingBox;
import geometry.Vector3D;

public final class Sphere extends RigidBody {
    Vector3D center;
    double radius;
    double mass;
    public Sphere(Vector3D center, double radius, double mass) {
        this.center = center;
        this.radius = radius;
        this.mass = mass;
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

    }
}
