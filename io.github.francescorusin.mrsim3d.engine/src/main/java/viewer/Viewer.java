package viewer;

import geometry.Vector3D;

public interface Viewer {
    void drawTriangle(Vector3D v1, Vector3D v2, Vector3D v3);
    void drawSphere(Vector3D position, double radius);
    void drawLine(Vector3D p1, Vector3D p2);
}
