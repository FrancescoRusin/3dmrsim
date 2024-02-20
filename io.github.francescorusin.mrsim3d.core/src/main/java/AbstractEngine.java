import geometry.Vector3D;

public abstract class AbstractEngine implements Engine {
    protected Vector3D DEFAULT_GRAVITY = new Vector3D(0d, 0d, -9.81);
    protected double time;
}
