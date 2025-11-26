/*-
 * ========================LICENSE_START=================================
 * mrsim3d.engine
 * %%
 * Copyright (C) 2024 - 2025 Francesco Rusin
 * %%
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * =========================LICENSE_END==================================
 */
package agents;

import bodies.*;
import engine.Ode4jEngine;
import geometry.BoundingBox;
import geometry.Vector3D;
import java.util.*;
import java.util.stream.Collectors;

import joints.FixedJoint;
import snapshot.*;
import utils.UnorderedPair;
import viewer.Viewer;

public abstract class AbstractGridRobot implements EmbodiedAgent {
  protected final Voxel[][][] grid;
  protected final double voxelSideLength;
  protected final double voxelMass;
  protected final Set<UnorderedPair<int[]>> intraVoxelLocks;

  private enum Cache {
    BBOX,
    POSITION,
    VELOCITY
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
    return Arrays.stream(grid)
            .flatMap(aa -> Arrays.stream(aa).flatMap(Arrays::stream))
            .filter(Objects::nonNull)
            .collect(Collectors.toList());
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
    double[] leftBackDownCenter =
            new double[]{
                    position.x() - voxelSideLength * (grid.length - 1) / 2,
                    position.y() - voxelSideLength * (grid[0].length - 1) / 2,
                    position.z() - voxelSideLength * (grid[0][0].length - 1) / 2
            };
    // assemble voxels
    for (int x = 0; x < grid.length; ++x) {
      for (int y = 0; y < grid[0].length; ++y) {
        for (int z = 0; z < grid[0][0].length; ++z) {
          if (Objects.nonNull(grid[x][y][z])) {
            grid[x][y][z].assemble(
                    engine,
                    new Vector3D(
                            leftBackDownCenter[0] + x * voxelSideLength,
                            leftBackDownCenter[1] + y * voxelSideLength,
                            leftBackDownCenter[2] + z * voxelSideLength));
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
              intraVoxelLocks.add(new UnorderedPair<>(new int[]{x - 1, y, z}, new int[]{x, y, z}));
              engine.addFixedJoint(
                      grid[x - 1][y][z].vertexBody(Voxel.Vertex.V100),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V000));
              engine.addFixedJoint(
                      grid[x - 1][y][z].vertexBody(Voxel.Vertex.V101),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V001));
              engine.addFixedJoint(
                      grid[x - 1][y][z].vertexBody(Voxel.Vertex.V110),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V010));
              engine.addFixedJoint(
                      grid[x - 1][y][z].vertexBody(Voxel.Vertex.V111),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V011));
            }
            if (y >= 1 && Objects.nonNull(grid[x][y - 1][z])) {
              intraVoxelLocks.add(new UnorderedPair<>(new int[]{x, y - 1, z}, new int[]{x, y, z}));
              engine.addFixedJoint(
                      grid[x][y - 1][z].vertexBody(Voxel.Vertex.V010),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V000));
              engine.addFixedJoint(
                      grid[x][y - 1][z].vertexBody(Voxel.Vertex.V011),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V001));
              engine.addFixedJoint(
                      grid[x][y - 1][z].vertexBody(Voxel.Vertex.V110),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V100));
              engine.addFixedJoint(
                      grid[x][y - 1][z].vertexBody(Voxel.Vertex.V111),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V101));
            }
            if (z >= 1 && Objects.nonNull(grid[x][y][z - 1])) {
              intraVoxelLocks.add(new UnorderedPair<>(new int[]{x, y, z - 1}, new int[]{x, y, z}));
              engine.addFixedJoint(
                      grid[x][y][z - 1].vertexBody(Voxel.Vertex.V001),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V000));
              engine.addFixedJoint(
                      grid[x][y][z - 1].vertexBody(Voxel.Vertex.V011),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V010));
              engine.addFixedJoint(
                      grid[x][y][z - 1].vertexBody(Voxel.Vertex.V101),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V100));
              engine.addFixedJoint(
                      grid[x][y][z - 1].vertexBody(Voxel.Vertex.V111),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V110));
            }
            if (x >= 1 && y >= 1 && Objects.nonNull(grid[x - 1][y - 1][z])) {
              intraVoxelLocks.add(new UnorderedPair<>(new int[]{x - 1, y - 1, z}, new int[]{x, y, z}));
              engine.addFixedJoint(
                      grid[x - 1][y - 1][z].vertexBody(Voxel.Vertex.V110),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V000));
              engine.addFixedJoint(
                      grid[x - 1][y - 1][z].vertexBody(Voxel.Vertex.V111),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V001));
            }
            if (x >= 1 && z >= 1 && Objects.nonNull(grid[x - 1][y][z - 1])) {
              intraVoxelLocks.add(new UnorderedPair<>(new int[]{x - 1, y, z - 1}, new int[]{x, y, z}));
              engine.addFixedJoint(
                      grid[x - 1][y][z - 1].vertexBody(Voxel.Vertex.V101),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V000));
              engine.addFixedJoint(
                      grid[x - 1][y][z - 1].vertexBody(Voxel.Vertex.V111),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V010));
            }
            if (y >= 1 && z >= 1 && Objects.nonNull(grid[x][y - 1][z - 1])) {
              intraVoxelLocks.add(new UnorderedPair<>(new int[]{x, y - 1, z - 1}, new int[]{x, y, z}));
              engine.addFixedJoint(
                      grid[x][y - 1][z - 1].vertexBody(Voxel.Vertex.V011),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V000));
              engine.addFixedJoint(
                      grid[x][y - 1][z - 1].vertexBody(Voxel.Vertex.V111),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V100));
            }
            if (x >= 1 && y >= 1 && z >= 1 && Objects.nonNull(grid[x - 1][y - 1][z - 1])) {
              intraVoxelLocks.add(new UnorderedPair<>(new int[]{x - 1, y - 1, z - 1}, new int[]{x, y, z}));
              engine.addFixedJoint(
                      grid[x - 1][y - 1][z - 1].vertexBody(Voxel.Vertex.V111),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V000));
            }
            if (x >= 1 && y < grid[0].length - 1 && Objects.nonNull(grid[x - 1][y + 1][z])) {
              intraVoxelLocks.add(new UnorderedPair<>(new int[]{x - 1, y + 1, z}, new int[]{x, y, z}));
              engine.addFixedJoint(
                      grid[x - 1][y + 1][z].vertexBody(Voxel.Vertex.V100),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V010));
              engine.addFixedJoint(
                      grid[x - 1][y + 1][z].vertexBody(Voxel.Vertex.V101),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V011));
            }
            if (x >= 1 && z < grid[0][0].length - 1 && Objects.nonNull(grid[x - 1][y][z + 1])) {
              intraVoxelLocks.add(new UnorderedPair<>(new int[]{x - 1, y, z + 1}, new int[]{x, y, z}));
              engine.addFixedJoint(
                      grid[x - 1][y][z + 1].vertexBody(Voxel.Vertex.V100),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V001));
              engine.addFixedJoint(
                      grid[x - 1][y][z + 1].vertexBody(Voxel.Vertex.V110),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V011));
            }
            if (y >= 1 && z < grid[0][0].length - 1 && Objects.nonNull(grid[x][y - 1][z + 1])) {
              intraVoxelLocks.add(new UnorderedPair<>(new int[]{x, y - 1, z + 1}, new int[]{x, y, z}));
              engine.addFixedJoint(
                      grid[x][y - 1][z + 1].vertexBody(Voxel.Vertex.V010),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V001));
              engine.addFixedJoint(
                      grid[x][y - 1][z + 1].vertexBody(Voxel.Vertex.V110),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V101));
            }
            if (x >= 1
                    && y >= 1
                    && z < grid[0][0].length - 1
                    && Objects.nonNull(grid[x - 1][y - 1][z + 1])) {
              intraVoxelLocks.add(new UnorderedPair<>(new int[]{x - 1, y - 1, z + 1}, new int[]{x, y, z}));
              engine.addFixedJoint(
                      grid[x - 1][y - 1][z + 1].vertexBody(Voxel.Vertex.V110),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V001));
            }
            if (x >= 1
                    && y < grid[0].length - 1
                    && z < grid[0][0].length - 1
                    && Objects.nonNull(grid[x - 1][y + 1][z + 1])) {
              intraVoxelLocks.add(new UnorderedPair<>(new int[]{x - 1, y + 1, z + 1}, new int[]{x, y, z}));
              engine.addFixedJoint(
                      grid[x - 1][y + 1][z + 1].vertexBody(Voxel.Vertex.V100),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V011));
            }
            if (x < grid.length - 1
                    && y < grid[0].length - 1
                    && z < grid[0][0].length - 1
                    && Objects.nonNull(grid[x + 1][y + 1][z + 1])) {
              intraVoxelLocks.add(new UnorderedPair<>(new int[]{x + 1, y + 1, z + 1}, new int[]{x, y, z}));
              engine.addFixedJoint(
                      grid[x + 1][y + 1][z + 1].vertexBody(Voxel.Vertex.V000),
                      grid[x][y][z].vertexBody(Voxel.Vertex.V111));
            }
          }
        }
      }
    }
  }

  @Override
  public void cacheAction(ActionSnapshot actionSnapshot) {
    //TODO IMPLEMENT
  }

  public interface GridRobotSnapshot extends MultibodySnapshot {
    Voxel.VoxelSnapshot[][][] grid();

    @Override
    default List<BodySnapshot> bodyParts() {
      return Arrays.stream(grid())
              .flatMap(aa -> Arrays.stream(aa).flatMap(Arrays::stream))
              .filter(Objects::nonNull)
              .collect(Collectors.toList());
    }

    default void drawGrid(Viewer viewer) {
      for (Voxel.VoxelSnapshot[][] voxelPlane : grid()) {
        for (Voxel.VoxelSnapshot[] row : voxelPlane) {
          for (Voxel.VoxelSnapshot voxel : row) {
            if (Objects.nonNull(voxel)) {
              voxel.draw(viewer);
            }
          }
        }
      }
    }
  }

  public record GridRobotSnapshotComputation(Voxel.VoxelSnapshotBase[][][] grid) implements GridRobotSnapshot {
    @Override
    public void draw(Viewer viewer) {
      throw new RuntimeException("Attempted to draw using computation only class");
    }
  }

  public record GridRobotSnapshotDebug(
          Voxel.VoxelSnapshotDebug[][][] grid,
          HashSet<UnorderedPair<int[]>> intraVoxelLocks,
          List<ActionSnapshot> actions
  ) implements GridRobotSnapshot {
    public List<JointSnapshot> internalJoints() {
      return intraVoxelLocks.stream().map(p -> {
        List<Voxel.VoxelSnapshotDebug> voxels = p.elements().stream().map(e -> this.grid[e[0]][e[1]][e[2]]).toList();
        return (JointSnapshot) new FixedJoint.FixedJointSnapshot(voxels.get(0).position(), voxels.get(1).position());
      }).toList();
    }

    @Override
    public void draw(Viewer viewer) {
      drawGrid(viewer);
      //TODO ACTIONS!
    }
  }

  public record GridRobotSnapshotDisplay(Voxel.VoxelSnapshotBase[][][] grid, List<ActionSnapshot> actions) implements GridRobotSnapshot {
    @Override
    public void draw(Viewer viewer) {
      drawGrid(viewer);
      //TODO ACTIONS!
    }
  }

  @Override
  public BodySnapshot snapshot(Ode4jEngine engine, Ode4jEngine.Mode mode) {
    switch (mode) {
      case COMPUTATION: {
        Voxel.VoxelSnapshotBase[][][] snapshotGrid = new Voxel.VoxelSnapshotBase[grid.length][grid[0].length][grid[0][0].length];
        for (int x = 0; x < grid.length; ++x) {
          for (int y = 0; y < grid[0].length; ++y) {
            for (int z = 0; z < grid[0][0].length; ++z) {
              if (Objects.nonNull(grid[x][y][z])) {
                snapshotGrid[x][y][z] = (Voxel.VoxelSnapshotBase) grid[x][y][z].snapshot(engine, Ode4jEngine.Mode.COMPUTATION);
              }
            }
          }
        }
        return new GridRobotSnapshotComputation(snapshotGrid);
      }
      case DISPLAY: {
        Voxel.VoxelSnapshotBase[][][] snapshotGrid = new Voxel.VoxelSnapshotBase[grid.length][grid[0].length][grid[0][0].length];
        for (int x = 0; x < grid.length; ++x) {
          for (int y = 0; y < grid[0].length; ++y) {
            for (int z = 0; z < grid[0][0].length; ++z) {
              if (Objects.nonNull(grid[x][y][z])) {
                snapshotGrid[x][y][z] = (Voxel.VoxelSnapshotBase) grid[x][y][z].snapshot(engine, Ode4jEngine.Mode.DISPLAY);
              }
            }
          }
        }
        return new GridRobotSnapshotDisplay(snapshotGrid, List.of()); // TODO ACTIONS!
      }
      case DEBUG: {
        Voxel.VoxelSnapshotDebug[][][] snapshotGrid = new Voxel.VoxelSnapshotDebug[grid.length][grid[0].length][grid[0][0].length];
        for (int x = 0; x < grid.length; ++x) {
          for (int y = 0; y < grid[0].length; ++y) {
            for (int z = 0; z < grid[0][0].length; ++z) {
              if (Objects.nonNull(grid[x][y][z])) {
                snapshotGrid[x][y][z] = (Voxel.VoxelSnapshotDebug) grid[x][y][z].snapshot(engine, Ode4jEngine.Mode.DEBUG);
              }
            }
          }
        }
        return new GridRobotSnapshotDebug(snapshotGrid, new HashSet<>(intraVoxelLocks), List.of()); // TODO ACTIONS!
      }
    }
    throw new AssertionError("Problem with a switch: this point should be unreachable");
  }
}