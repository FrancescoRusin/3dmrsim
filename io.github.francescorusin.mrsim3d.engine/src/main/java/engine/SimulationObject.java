package engine;

import geometry.BoundingBox;
import geometry.Vector3D;

public interface SimulationObject {
    BoundingBox boundingBox(double t);

    Vector3D position(double t);

    Vector3D velocity(double t);

    void rotate(Ode4jEngine engine, Vector3D eulerAngles);

    void translate(Ode4jEngine engine, Vector3D translation);

    void assemble(Ode4jEngine engine, Vector3D position);
}