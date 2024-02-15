import geometry.BoundingBox;
import geometry.Vector3D;

import java.util.List;

public abstract class MultiBody implements Body {
    @Override
    public BoundingBox boundingBox() {
        return getBodyParts().stream().map(Body::boundingBox).reduce(BoundingBox::enclosing).orElseThrow();
    }
    
    abstract public List<Body> getBodyParts();

    @Override
    public double mass() {
        return getBodyParts().stream().mapToDouble(Body::mass).sum();
    }

    @Override
    public Vector3D position() {
        return getBodyParts().stream().map(b -> b.position().times(b.mass())).reduce(Vector3D::sum).orElseThrow();
    }

    @Override
    public Vector3D velocity() {
        return getBodyParts().stream().map(b -> b.velocity().times(b.mass())).reduce(Vector3D::sum).orElseThrow();
    }
}
