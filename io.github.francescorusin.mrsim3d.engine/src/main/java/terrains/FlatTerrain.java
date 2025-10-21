package terrains;

import geometry.Vector3D;
import org.ode4j.ode.OdeHelper;
import viewer.Viewer;

import java.util.Objects;

public class FlatTerrain extends Terrain {
    Integer texID;
    public FlatTerrain() {
        super(dSpace -> OdeHelper.createPlane(dSpace, 0, 0, 1, 0));
    }

    @Override
    public void draw(Viewer viewer) {
        if (Objects.isNull(texID)) {
            texID = viewer.loadTexture("C:\\Users\\Francesco\\IdeaProjects\\3dmrsim\\io.github.francescorusin.mrsim3d.engine\\src\\main\\resources\\textures\\Chessboard.jpg");
        }
        viewer.drawTexture(
                new Vector3D(-100, -100, 0), new Vector3D(100, -100, 0),
                new Vector3D(100, 100, 0), new Vector3D(-100, 100, 0),
                texID, 1, 1);
    }
}
