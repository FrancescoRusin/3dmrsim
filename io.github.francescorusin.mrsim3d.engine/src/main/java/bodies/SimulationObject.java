package bodies;

import engine.Ode4jEngine;
import geometry.BoundingBox;
import geometry.Vector3D;
import snapshot.ObjectSnapshot;

import java.util.List;

public interface SimulationObject {
    List<Body> bodyParts();
    BoundingBox boundingBox(double t);

    Vector3D position(double t);

    Vector3D velocity(double t);

    void rotate(Ode4jEngine engine, Vector3D eulerAngles);

    void translate(Ode4jEngine engine, Vector3D translation);

    void assemble(Ode4jEngine engine, Vector3D position);
    //TODO IMPLEMENT
    ObjectSnapshot snapshot(Ode4jEngine engine);
}