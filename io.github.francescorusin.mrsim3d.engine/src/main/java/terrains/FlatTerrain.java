package terrains;

import geometry.Vector3D;
import org.ode4j.ode.OdeHelper;
import viewer.Viewer;

public class FlatTerrain extends Terrain {
    public FlatTerrain() {
        super(dSpace -> OdeHelper.createPlane(dSpace, 0, 0, 1, 0));
    }

    @Override
    public void draw(Viewer viewer) {
        viewer.drawTriangle(new Vector3D(-100, -100, 0), new Vector3D(100, -100, 0), new Vector3D(100, 100, 0), TERRAIN_COLOR);
        viewer.drawTriangle(new Vector3D(-100, -100, 0), new Vector3D(-100, 100, 0), new Vector3D(100, 100, 0), TERRAIN_COLOR);
    }
}
