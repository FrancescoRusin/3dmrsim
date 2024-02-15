import geometry.Vector3D;
import org.ode4j.ode.DBody;

public abstract class RigidBody implements Body {
    DBody body;

    @Override
    public Vector3D position() {
        return new Vector3D(body.getPosition().get0(), body.getPosition().get1(), body.getPosition().get2());
    }

    @Override
    public Vector3D velocity() {
        return new Vector3D(body.getLinearVel().get0(), body.getLinearVel().get1(), body.getLinearVel().get2());
    }

    public DBody getBody() {
        return body;
    }

    public void setBody(DBody body) {
        this.body = body;
    }
}
