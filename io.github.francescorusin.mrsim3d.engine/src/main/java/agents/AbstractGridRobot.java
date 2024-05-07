package agents;

import actions.Action;
import bodies.*;
import engine.Ode4jEngine;
import geometry.BoundingBox;
import geometry.Vector3D;
import outcome.AgentSnapshot;
import outcome.ObjectSnapshot;
import utils.UnorderedPair;
import viewer.Viewer;

import java.util.*;
import java.util.stream.Collectors;

public abstract class AbstractGridRobot implements EmbodiedAgent {
    protected final Voxel[][][] grid;
    protected final double voxelSideLength;
    protected final double voxelMass;
    protected final Set<UnorderedPair<Voxel>> intraVoxelLocks;

    private enum Cache {
        BBOX, POSITION, VELOCITY
    }

    private final EnumMap<Cache, Double> cacheTime;
    private final EnumMap<Cache, Object> cacher;

    public AbstractGridRobot(Voxel[][][] grid, double voxelSideLength, double voxelMass) {
        this.grid = grid;
        this.voxelSideLength = voxelSideLength;
        this.voxelMass = voxelMass;
        this.intraVoxelLocks = new HashSet<>(grid.length * grid[0].length * grid[0][0].length * 6);
        this.cacheTime = new EnumMap<>(Cache.class);
        this.cacher = new EnumMap<>(Cache.class);
    }

    @Override
    public List<AbstractBody> components() {
        return Arrays.stream(grid).flatMap(aa -> Arrays.stream(aa).flatMap(Arrays::stream))
                .filter(Objects::nonNull).collect(Collectors.toList());
    }

    @Override
    public BoundingBox boundingBox(double t) {
        if (cacheTime.get(Cache.BBOX) != t) {
            cacheTime.put(Cache.BBOX, t);
            cacher.put(Cache.BBOX, EmbodiedAgent.super.boundingBox(t));
        }
        return (BoundingBox) cacher.get(Cache.BBOX);
    }

    @Override
    public Vector3D position(double t) {
        if (cacheTime.get(Cache.POSITION) != t) {
            cacheTime.put(Cache.POSITION, t);
            cacher.put(Cache.POSITION, EmbodiedAgent.super.position(t));
        }
        return (Vector3D) cacher.get(Cache.POSITION);
    }

    @Override
    public Vector3D velocity(double t) {
        if (cacheTime.get(Cache.VELOCITY) != t) {
            cacheTime.put(Cache.VELOCITY, t);
            cacher.put(Cache.VELOCITY, EmbodiedAgent.super.velocity(t));
        }
        return (Vector3D) cacher.get(Cache.VELOCITY);
    }

    @Override
    public void rotate(Ode4jEngine engine, Vector3D eulerAngles) {
        EmbodiedAgent.super.rotate(engine, eulerAngles);
        cacheTime.put(Cache.BBOX, -1d);
        cacheTime.put(Cache.VELOCITY, -1d);
    }

    @Override
    public void translate(Ode4jEngine engine, Vector3D translation) {
        EmbodiedAgent.super.translate(engine, translation);
        cacheTime.put(Cache.BBOX, -1d);
        cacheTime.put(Cache.POSITION, -1d);
    }

    @Override
    public void assemble(Ode4jEngine engine, Vector3D position) {
        for (Cache cache : Cache.values()) {
            cacheTime.put(cache, -1d);
        }
        double[] leftBackDownCenter = new double[]{
                position.x() - voxelSideLength * (grid.length - 1) / 2,
                position.y() - voxelSideLength * (grid[0].length - 1) / 2,
                position.z() - voxelSideLength * (grid[0][0].length - 1) / 2};
        // assemble voxels
        for (int x = 0; x < grid.length; ++x) {
            for (int y = 0; y < grid[0].length; ++y) {
                for (int z = 0; z < grid[0][0].length; ++z) {
                    if (Objects.nonNull(grid[x][y][z])) {
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
                    if (Objects.nonNull(grid[x][y][z])) {
                        if (x >= 1 && Objects.nonNull(grid[x - 1][y][z])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x - 1][y][z], grid[x][y][z]));
                            engine.addFixedJoint(grid[x - 1][y][z].vertexBody(Voxel.Vertex.V100), grid[x][y][z].vertexBody(Voxel.Vertex.V000));
                            engine.addFixedJoint(grid[x - 1][y][z].vertexBody(Voxel.Vertex.V101), grid[x][y][z].vertexBody(Voxel.Vertex.V001));
                            engine.addFixedJoint(grid[x - 1][y][z].vertexBody(Voxel.Vertex.V110), grid[x][y][z].vertexBody(Voxel.Vertex.V010));
                            engine.addFixedJoint(grid[x - 1][y][z].vertexBody(Voxel.Vertex.V111), grid[x][y][z].vertexBody(Voxel.Vertex.V011));
                        }
                        if (y >= 1 && Objects.nonNull(grid[x][y - 1][z])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x][y - 1][z], grid[x][y][z]));
                            engine.addFixedJoint(grid[x][y - 1][z].vertexBody(Voxel.Vertex.V010), grid[x][y][z].vertexBody(Voxel.Vertex.V000));
                            engine.addFixedJoint(grid[x][y - 1][z].vertexBody(Voxel.Vertex.V011), grid[x][y][z].vertexBody(Voxel.Vertex.V001));
                            engine.addFixedJoint(grid[x][y - 1][z].vertexBody(Voxel.Vertex.V110), grid[x][y][z].vertexBody(Voxel.Vertex.V100));
                            engine.addFixedJoint(grid[x][y - 1][z].vertexBody(Voxel.Vertex.V111), grid[x][y][z].vertexBody(Voxel.Vertex.V101));
                        }
                        if (z >= 1 && Objects.nonNull(grid[x][y][z - 1])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x][y][z - 1], grid[x][y][z]));
                            engine.addFixedJoint(grid[x][y][z - 1].vertexBody(Voxel.Vertex.V001), grid[x][y][z].vertexBody(Voxel.Vertex.V000));
                            engine.addFixedJoint(grid[x][y][z - 1].vertexBody(Voxel.Vertex.V011), grid[x][y][z].vertexBody(Voxel.Vertex.V010));
                            engine.addFixedJoint(grid[x][y][z - 1].vertexBody(Voxel.Vertex.V101), grid[x][y][z].vertexBody(Voxel.Vertex.V100));
                            engine.addFixedJoint(grid[x][y][z - 1].vertexBody(Voxel.Vertex.V111), grid[x][y][z].vertexBody(Voxel.Vertex.V110));
                        }
                        if (x >= 1 && y >= 1 && Objects.nonNull(grid[x - 1][y - 1][z])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x - 1][y - 1][z], grid[x][y][z]));
                            engine.addFixedJoint(grid[x - 1][y - 1][z].vertexBody(Voxel.Vertex.V110), grid[x][y][z].vertexBody(Voxel.Vertex.V000));
                            engine.addFixedJoint(grid[x - 1][y - 1][z].vertexBody(Voxel.Vertex.V111), grid[x][y][z].vertexBody(Voxel.Vertex.V001));
                        }
                        if (x >= 1 && z >= 1 && Objects.nonNull(grid[x - 1][y][z - 1])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x - 1][y][z - 1], grid[x][y][z]));
                            engine.addFixedJoint(grid[x - 1][y][z - 1].vertexBody(Voxel.Vertex.V101), grid[x][y][z].vertexBody(Voxel.Vertex.V000));
                            engine.addFixedJoint(grid[x - 1][y][z - 1].vertexBody(Voxel.Vertex.V111), grid[x][y][z].vertexBody(Voxel.Vertex.V010));
                        }
                        if (y >= 1 && z >= 1 && Objects.nonNull(grid[x][y - 1][z - 1])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x][y - 1][z - 1], grid[x][y][z]));
                            engine.addFixedJoint(grid[x][y - 1][z - 1].vertexBody(Voxel.Vertex.V011), grid[x][y][z].vertexBody(Voxel.Vertex.V000));
                            engine.addFixedJoint(grid[x][y - 1][z - 1].vertexBody(Voxel.Vertex.V111), grid[x][y][z].vertexBody(Voxel.Vertex.V100));
                        }
                        if (x >= 1 && y >= 1 && z >= 1 && Objects.nonNull(grid[x - 1][y - 1][z - 1])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x - 1][y - 1][z - 1], grid[x][y][z]));
                            engine.addFixedJoint(grid[x - 1][y - 1][z - 1].vertexBody(Voxel.Vertex.V111), grid[x][y][z].vertexBody(Voxel.Vertex.V000));
                        }
                        if (x >= 1 && y < grid[0].length - 1 && Objects.nonNull(grid[x - 1][y + 1][z])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x - 1][y + 1][z], grid[x][y][z]));
                            engine.addFixedJoint(grid[x - 1][y + 1][z].vertexBody(Voxel.Vertex.V100), grid[x][y][z].vertexBody(Voxel.Vertex.V010));
                            engine.addFixedJoint(grid[x - 1][y + 1][z].vertexBody(Voxel.Vertex.V101), grid[x][y][z].vertexBody(Voxel.Vertex.V011));
                        }
                        if (x >= 1 && z < grid[0][0].length - 1 && Objects.nonNull(grid[x - 1][y][z + 1])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x - 1][y][z + 1], grid[x][y][z]));
                            engine.addFixedJoint(grid[x - 1][y][z + 1].vertexBody(Voxel.Vertex.V100), grid[x][y][z].vertexBody(Voxel.Vertex.V001));
                            engine.addFixedJoint(grid[x - 1][y][z + 1].vertexBody(Voxel.Vertex.V110), grid[x][y][z].vertexBody(Voxel.Vertex.V011));
                        }
                        if (y >= 1 && z < grid[0][0].length - 1 && Objects.nonNull(grid[x][y - 1][z + 1])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x][y - 1][z + 1], grid[x][y][z]));
                            engine.addFixedJoint(grid[x][y - 1][z + 1].vertexBody(Voxel.Vertex.V010), grid[x][y][z].vertexBody(Voxel.Vertex.V001));
                            engine.addFixedJoint(grid[x][y - 1][z + 1].vertexBody(Voxel.Vertex.V110), grid[x][y][z].vertexBody(Voxel.Vertex.V101));
                        }
                        if (x >= 1 && y >= 1 && z < grid[0][0].length - 1 && Objects.nonNull(grid[x - 1][y - 1][z + 1])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x - 1][y - 1][z + 1], grid[x][y][z]));
                            engine.addFixedJoint(grid[x - 1][y - 1][z + 1].vertexBody(Voxel.Vertex.V110), grid[x][y][z].vertexBody(Voxel.Vertex.V001));
                        }
                        if (x >= 1 && y < grid[0].length - 1 && z < grid[0][0].length - 1 && Objects.nonNull(grid[x - 1][y + 1][z + 1])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x - 1][y + 1][z + 1], grid[x][y][z]));
                            engine.addFixedJoint(grid[x - 1][y + 1][z + 1].vertexBody(Voxel.Vertex.V100), grid[x][y][z].vertexBody(Voxel.Vertex.V011));
                        }
                        if (x < grid.length - 1 && y < grid[0].length - 1 && z < grid[0][0].length - 1 && Objects.nonNull(grid[x + 1][y + 1][z + 1])) {
                            intraVoxelLocks.add(new UnorderedPair<>(grid[x + 1][y + 1][z + 1], grid[x][y][z]));
                            engine.addFixedJoint(grid[x + 1][y + 1][z + 1].vertexBody(Voxel.Vertex.V000), grid[x][y][z].vertexBody(Voxel.Vertex.V111));
                        }
                    }
                }
            }
        }
    }

    public record RobotSnapshot(Voxel.VoxelSnapshot[][][] voxelSnapshots,
                                List<Action>[][][] lastStepActions) implements AgentSnapshot {
        public RobotSnapshot(Voxel.VoxelSnapshot[][][] voxelSnapshots) {
            this(voxelSnapshots, new List[voxelSnapshots.length][voxelSnapshots[0].length][voxelSnapshots[0][0].length]);
            for (int i = 0; i < voxelSnapshots.length; ++i) {
                for (int j = 0; j < voxelSnapshots[0].length; ++j) {
                    for (int k = 0; k < voxelSnapshots[0][0].length; ++k) {
                        if (Objects.nonNull(voxelSnapshots[i][j][k])) {
                            this.lastStepActions[i][j][k] = List.of();
                        }
                    }
                }
            }
        }

        @Override
        public List<ObjectSnapshot> components() {
            return Arrays.stream(voxelSnapshots).flatMap(aa -> Arrays.stream(aa).flatMap(Arrays::stream))
                    .filter(Objects::nonNull).collect(Collectors.toList());
        }

        @Override
        public void draw(Viewer viewer) {
            //TODO IMPLEMENT
        }
    }

    @Override
    public AgentSnapshot snapshot(Ode4jEngine engine) {
        Voxel.VoxelSnapshot[][][] snapshotGrid = new Voxel.VoxelSnapshot[grid.length][grid[0].length][grid[0][0].length];
        for (int i = 0; i < grid.length; ++i) {
            for (int j = 0; j < grid[0].length; ++j) {
                for (int k = 0; k < grid[0][0].length; ++k) {
                    snapshotGrid[i][j][k] = (Voxel.VoxelSnapshot) grid[i][j][k].snapshot(engine);
                }
            }
        }
        return new RobotSnapshot(snapshotGrid);
    }
}