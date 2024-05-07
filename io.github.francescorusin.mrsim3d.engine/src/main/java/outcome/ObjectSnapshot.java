package outcome;

import geometry.Vector3D;
import viewer.Viewer;

public interface ObjectSnapshot {
    Vector3D position();
    void draw(Viewer viewer);
}
