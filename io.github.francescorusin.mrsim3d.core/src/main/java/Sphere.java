import geometry.BoundingBox;
import geometry.Vector3D;

public abstract class Sphere implements Body {
    private Vector3D center;
    private double radius;
    public Sphere(Vector3D center, double radius) {
        this.center = center;
        this.radius = radius;
    }

    @Override
    public BoundingBox boundingBox() {
        return new BoundingBox(
                new Vector3D(center.x() - radius, center.y() - radius, center.z() - radius),
                new Vector3D(center.x() + radius, center.y() + radius, center.z() + radius));
    }

    @Override
    public double volume() {
        return (double) 4 / 3 * Math.PI * Math.pow(radius, 3);
    }
}
