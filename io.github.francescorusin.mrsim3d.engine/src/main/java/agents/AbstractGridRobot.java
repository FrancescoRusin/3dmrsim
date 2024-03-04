package agents;

import bodies.AbstractBody;
import bodies.Body;
import bodies.Sphere;
import bodies.Voxel;
import drawstuff.DrawStuff;
import engine.Ode4jEngine;
import geometry.Vector3D;
import test.VisualTest;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

import static drawstuff.DrawStuff.*;

public abstract class AbstractGridRobot implements EmbodiedAgent {
    protected final Voxel[][][] grid;
    protected final double voxelSideLength;
    protected final double voxelMass;

    public AbstractGridRobot(Voxel[][][] grid, double voxelSideLength, double voxelMass) {
        this.grid = grid;
        this.voxelSideLength = voxelSideLength;
        this.voxelMass = voxelMass;
    }

    @Override
    public List<AbstractBody> getComponents() {
        return Arrays.stream(grid).flatMap(aa -> Arrays.stream(aa).flatMap(Arrays::stream)).collect(Collectors.toList());
    }

    @Override
    public void assemble(Ode4jEngine engine, Vector3D position) {
        double[] leftBackDownCenter = new double[]{
                position.x() - voxelSideLength * (grid.length - 1) / 2,
                position.y() - voxelSideLength * (grid[0].length - 1) / 2,
                position.z() - voxelSideLength * (grid[0][0].length - 1) / 2};
        for (int x = 0; x < grid.length; ++x) {
            for (int y = 0; y < grid[0].length; ++y) {
                for (int z = 0; z < grid[0][0].length; ++z) {
                    if (!Objects.isNull(grid[x][y][z])) {
                        grid[x][y][z].assemble(engine,
                                new Vector3D(
                                        leftBackDownCenter[0] + x * voxelSideLength,
                                        leftBackDownCenter[1] + y * voxelSideLength,
                                        leftBackDownCenter[2] + z * voxelSideLength
                                ));
                        if (x >= 1 && !Objects.isNull(grid[x - 1][y][z])) {
                            engine.addRigidJoint(grid[x - 1][y][z].getVertexBody(Voxel.Vertex.V100), grid[x][y][z].getVertexBody(Voxel.Vertex.V000));
                            engine.addRigidJoint(grid[x - 1][y][z].getVertexBody(Voxel.Vertex.V101), grid[x][y][z].getVertexBody(Voxel.Vertex.V001));
                            engine.addRigidJoint(grid[x - 1][y][z].getVertexBody(Voxel.Vertex.V110), grid[x][y][z].getVertexBody(Voxel.Vertex.V010));
                            engine.addRigidJoint(grid[x - 1][y][z].getVertexBody(Voxel.Vertex.V111), grid[x][y][z].getVertexBody(Voxel.Vertex.V011));
                        }
                        if (y >= 1 && !Objects.isNull(grid[x][y - 1][z])) {
                            engine.addRigidJoint(grid[x][y - 1][z].getVertexBody(Voxel.Vertex.V010), grid[x][y][z].getVertexBody(Voxel.Vertex.V000));
                            engine.addRigidJoint(grid[x][y - 1][z].getVertexBody(Voxel.Vertex.V011), grid[x][y][z].getVertexBody(Voxel.Vertex.V001));
                            engine.addRigidJoint(grid[x][y - 1][z].getVertexBody(Voxel.Vertex.V110), grid[x][y][z].getVertexBody(Voxel.Vertex.V100));
                            engine.addRigidJoint(grid[x][y - 1][z].getVertexBody(Voxel.Vertex.V111), grid[x][y][z].getVertexBody(Voxel.Vertex.V101));
                        }
                        if (z >= 1 && !Objects.isNull(grid[x][y][z - 1])) {
                            engine.addRigidJoint(grid[x][y][z - 1].getVertexBody(Voxel.Vertex.V001), grid[x][y][z].getVertexBody(Voxel.Vertex.V000));
                            engine.addRigidJoint(grid[x][y][z - 1].getVertexBody(Voxel.Vertex.V011), grid[x][y][z].getVertexBody(Voxel.Vertex.V010));
                            engine.addRigidJoint(grid[x][y][z - 1].getVertexBody(Voxel.Vertex.V101), grid[x][y][z].getVertexBody(Voxel.Vertex.V100));
                            engine.addRigidJoint(grid[x][y][z - 1].getVertexBody(Voxel.Vertex.V111), grid[x][y][z].getVertexBody(Voxel.Vertex.V110));
                        }
                    }
                }
            }
        }
    }

    @Override
    public void draw(VisualTest test) {
        dsSetColor(1, 1, 0);
        dsSetTexture(DrawStuff.DS_TEXTURE_NUMBER.DS_WOOD);
        for (int x = 0; x < grid.length; ++x) {
            for (int y = 0; y < grid[0].length; ++y) {
                for (int z = 0; z < grid[0][0].length; ++z) {
                    if (!Objects.isNull(grid[x][y][z])) {
                        for (Body body : grid[x][y][z].bodyParts()) {
                            dsDrawSphere(
                                    body.getBody().getPosition(),
                                    body.getBody().getRotation(),
                                    ((Sphere) body).getRadius());
                        }
                    }
                }
            }
        }
    }
}