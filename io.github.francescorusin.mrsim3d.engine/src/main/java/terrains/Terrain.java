package terrains;

import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSpace;
import viewer.Viewer;

import java.awt.*;
import java.util.function.Function;

public abstract class Terrain {
    protected final static Color TERRAIN_COLOR = new Color(255,235,205);
    private final Function<DSpace, DGeom> terrainGenerator;

    public Terrain(Function<DSpace, DGeom> terrainGenerator) {
        this.terrainGenerator = terrainGenerator;
    }

    public final DGeom generate(DSpace space) {
        return terrainGenerator.apply(space);
    }

    public abstract void draw(Viewer viewer);
}
