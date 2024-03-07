package agents;

import bodies.*;
import engine.Ode4jEngine;
import geometry.Vector3D;
import test.VisualTest;
import utils.UnorderedPair;

import java.util.*;
import java.util.stream.Collectors;

public abstract class AbstractGridRobot implements EmbodiedAgent {
    protected final Voxel[][][] grid;
    protected final double voxelSideLength;
    protected final double voxelMass;
    protected final Set<UnorderedPair<Voxel>> intraVoxelLocks;

    public AbstractGridRobot(Voxel[][][] grid, double voxelSideLength, double voxelMass) {
        this.grid = grid;
        this.voxelSideLength = voxelSideLength;
        this.voxelMass = voxelMass;
        this.intraVoxelLocks = new HashSet<>(grid.length * grid[0].length * grid[0][0].length * 6);
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
        // assemble voxels
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
                    }
                }
            }
        }
        // build intravoxel joints
        for (int x = 0; x < grid.length; ++x) {
            for (int y = 0; y < grid[0].length; ++y) {
                for (int z = 0; z < grid[0][0].length; ++z) {
                    if (!Objects.isNull(grid[x][y][z])) {
                        if (x >= 1 && !Objects.isNull(grid[x - 1][y][z])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x - 1][y][z], grid[x][y][z]));
                            engine.addFixedJoint(grid[x - 1][y][z].getVertexBody(Voxel.Vertex.V100), grid[x][y][z].getVertexBody(Voxel.Vertex.V000));
                            engine.addFixedJoint(grid[x - 1][y][z].getVertexBody(Voxel.Vertex.V101), grid[x][y][z].getVertexBody(Voxel.Vertex.V001));
                            engine.addFixedJoint(grid[x - 1][y][z].getVertexBody(Voxel.Vertex.V110), grid[x][y][z].getVertexBody(Voxel.Vertex.V010));
                            engine.addFixedJoint(grid[x - 1][y][z].getVertexBody(Voxel.Vertex.V111), grid[x][y][z].getVertexBody(Voxel.Vertex.V011));
                        }
                        if (y >= 1 && !Objects.isNull(grid[x][y - 1][z])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x][y - 1][z], grid[x][y][z]));
                            engine.addFixedJoint(grid[x][y - 1][z].getVertexBody(Voxel.Vertex.V010), grid[x][y][z].getVertexBody(Voxel.Vertex.V000));
                            engine.addFixedJoint(grid[x][y - 1][z].getVertexBody(Voxel.Vertex.V011), grid[x][y][z].getVertexBody(Voxel.Vertex.V001));
                            engine.addFixedJoint(grid[x][y - 1][z].getVertexBody(Voxel.Vertex.V110), grid[x][y][z].getVertexBody(Voxel.Vertex.V100));
                            engine.addFixedJoint(grid[x][y - 1][z].getVertexBody(Voxel.Vertex.V111), grid[x][y][z].getVertexBody(Voxel.Vertex.V101));
                        }
                        if (z >= 1 && !Objects.isNull(grid[x][y][z - 1])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x][y][z - 1], grid[x][y][z]));
                            engine.addFixedJoint(grid[x][y][z - 1].getVertexBody(Voxel.Vertex.V001), grid[x][y][z].getVertexBody(Voxel.Vertex.V000));
                            engine.addFixedJoint(grid[x][y][z - 1].getVertexBody(Voxel.Vertex.V011), grid[x][y][z].getVertexBody(Voxel.Vertex.V010));
                            engine.addFixedJoint(grid[x][y][z - 1].getVertexBody(Voxel.Vertex.V101), grid[x][y][z].getVertexBody(Voxel.Vertex.V100));
                            engine.addFixedJoint(grid[x][y][z - 1].getVertexBody(Voxel.Vertex.V111), grid[x][y][z].getVertexBody(Voxel.Vertex.V110));
                        }
                        if (x >= 1 && y >= 1 && !Objects.isNull(grid[x - 1][y - 1][z])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x - 1][y - 1][z], grid[x][y][z]));
                            engine.addFixedJoint(grid[x - 1][y - 1][z].getVertexBody(Voxel.Vertex.V110), grid[x][y][z].getVertexBody(Voxel.Vertex.V000));
                            engine.addFixedJoint(grid[x - 1][y - 1][z].getVertexBody(Voxel.Vertex.V111), grid[x][y][z].getVertexBody(Voxel.Vertex.V001));
                        }
                        if (x >= 1 && z >= 1 && !Objects.isNull(grid[x - 1][y][z - 1])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x - 1][y][z - 1], grid[x][y][z]));
                            engine.addFixedJoint(grid[x - 1][y][z - 1].getVertexBody(Voxel.Vertex.V101), grid[x][y][z].getVertexBody(Voxel.Vertex.V000));
                            engine.addFixedJoint(grid[x - 1][y][z - 1].getVertexBody(Voxel.Vertex.V111), grid[x][y][z].getVertexBody(Voxel.Vertex.V010));
                        }
                        if (y >= 1 && z >= 1 && !Objects.isNull(grid[x][y - 1][z - 1])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x][y - 1][z - 1], grid[x][y][z]));
                            engine.addFixedJoint(grid[x][y - 1][z - 1].getVertexBody(Voxel.Vertex.V011), grid[x][y][z].getVertexBody(Voxel.Vertex.V000));
                            engine.addFixedJoint(grid[x][y - 1][z - 1].getVertexBody(Voxel.Vertex.V111), grid[x][y][z].getVertexBody(Voxel.Vertex.V100));
                        }
                        if (x >= 1 && y >= 1 && z >= 1 && !Objects.isNull(grid[x - 1][y - 1][z - 1])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x - 1][y - 1][z - 1], grid[x][y][z]));
                            engine.addFixedJoint(grid[x - 1][y - 1][z - 1].getVertexBody(Voxel.Vertex.V111), grid[x][y][z].getVertexBody(Voxel.Vertex.V000));
                        }
                        if (x >= 1 && y < grid[0].length - 1 && !Objects.isNull(grid[x - 1][y + 1][z])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x - 1][y + 1][z], grid[x][y][z]));
                            engine.addFixedJoint(grid[x - 1][y + 1][z].getVertexBody(Voxel.Vertex.V100), grid[x][y][z].getVertexBody(Voxel.Vertex.V010));
                            engine.addFixedJoint(grid[x - 1][y + 1][z].getVertexBody(Voxel.Vertex.V101), grid[x][y][z].getVertexBody(Voxel.Vertex.V011));
                        }
                        if (x >= 1 && z < grid[0][0].length - 1 && !Objects.isNull(grid[x - 1][y][z + 1])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x - 1][y][z + 1], grid[x][y][z]));
                            engine.addFixedJoint(grid[x - 1][y][z + 1].getVertexBody(Voxel.Vertex.V100), grid[x][y][z].getVertexBody(Voxel.Vertex.V001));
                            engine.addFixedJoint(grid[x - 1][y][z + 1].getVertexBody(Voxel.Vertex.V110), grid[x][y][z].getVertexBody(Voxel.Vertex.V011));
                        }
                        if (y >= 1 && z < grid[0][0].length - 1 && !Objects.isNull(grid[x][y - 1][z + 1])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x][y - 1][z + 1], grid[x][y][z]));
                            engine.addFixedJoint(grid[x][y - 1][z + 1].getVertexBody(Voxel.Vertex.V010), grid[x][y][z].getVertexBody(Voxel.Vertex.V001));
                            engine.addFixedJoint(grid[x][y - 1][z + 1].getVertexBody(Voxel.Vertex.V110), grid[x][y][z].getVertexBody(Voxel.Vertex.V101));
                        }
                        if (x >= 1 && y >= 1 && z < grid[0][0].length - 1 && !Objects.isNull(grid[x - 1][y - 1][z + 1])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x - 1][y - 1][z + 1], grid[x][y][z]));
                            engine.addFixedJoint(grid[x - 1][y - 1][z + 1].getVertexBody(Voxel.Vertex.V110), grid[x][y][z].getVertexBody(Voxel.Vertex.V001));
                        }
                        if (x >= 1 && y < grid[0].length - 1 && z < grid[0][0].length - 1 && !Objects.isNull(grid[x - 1][y + 1][z + 1])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x - 1][y + 1][z + 1], grid[x][y][z]));
                            engine.addFixedJoint(grid[x - 1][y + 1][z + 1].getVertexBody(Voxel.Vertex.V100), grid[x][y][z].getVertexBody(Voxel.Vertex.V011));
                        }
                        if (x < grid.length - 1 && y < grid[0].length - 1 && z < grid[0][0].length - 1 && !Objects.isNull(grid[x + 1][y + 1][z + 1])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x + 1][y + 1][z + 1], grid[x][y][z]));
                            engine.addFixedJoint(grid[x + 1][y + 1][z + 1].getVertexBody(Voxel.Vertex.V000), grid[x][y][z].getVertexBody(Voxel.Vertex.V111));
                        }
                    }
                }
            }
        }
    }

    @Override
    public void draw(VisualTest test) {
        //TODO DRAW RIGID JOINTS
        for (Voxel[][] voxelX : grid) {
            for (Voxel[] voxelY : voxelX) {
                for (Voxel voxel : voxelY) {
                    if (!Objects.isNull(voxel)) {
                        voxel.draw(test);
                    }
                }
            }
        }
    }
}