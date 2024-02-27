package bodies; /*-
                 * ========================LICENSE_START=================================
                 * mrsim3d.engine
                 * %%
                 * Copyright (C) 2024 Francesco Rusin
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

import actions.Action;
import engine.Ode4jEngine;
import geometry.Vector3D;
import java.util.*;
import java.util.stream.Stream;
import org.ode4j.ode.DDoubleBallJoint;
import org.ode4j.ode.DJoint;
import sensors.AngleSensor;
import sensors.Sensor;
import sensors.VolumeRatioSensor;
import utils.Pair;

public class Voxel extends MultiBody implements SoftBody {
  public enum Vertex {
    V000,
    V001,
    V010,
    V011,
    V100,
    V101,
    V110,
    V111
  }

  enum Side {
    UP(Vertex.V001, Vertex.V101, Vertex.V111, Vertex.V011),
    DOWN(Vertex.V000, Vertex.V010, Vertex.V110, Vertex.V100),
    FRONT(Vertex.V010, Vertex.V011, Vertex.V111, Vertex.V110),
    BACK(Vertex.V000, Vertex.V100, Vertex.V101, Vertex.V001),
    RIGHT(Vertex.V100, Vertex.V110, Vertex.V111, Vertex.V101),
    LEFT(Vertex.V000, Vertex.V001, Vertex.V011, Vertex.V010);
    private final Vertex v1;
    private final Vertex v2;
    private final Vertex v3;
    private final Vertex v4;

    Side(Vertex v1, Vertex v2, Vertex v3, Vertex v4) {
      this.v1 = v1;
      this.v2 = v2;
      this.v3 = v3;
      this.v4 = v4;
    }
  }

  enum Edge {
    DOWN_BACK(Vertex.V000, Vertex.V100),
    DOWN_RIGHT(Vertex.V100, Vertex.V110),
    DOWN_LEFT(Vertex.V000, Vertex.V010),
    DOWN_FRONT(Vertex.V010, Vertex.V110),
    UP_BACK(Vertex.V001, Vertex.V101),
    UP_RIGHT(Vertex.V101, Vertex.V111),
    UP_LEFT(Vertex.V001, Vertex.V011),
    UP_FRONT(Vertex.V011, Vertex.V111),
    SIDE_BR(Vertex.V100, Vertex.V101),
    SIDE_BL(Vertex.V000, Vertex.V001),
    SIDE_FR(Vertex.V110, Vertex.V111),
    SIDE_FL(Vertex.V010, Vertex.V011);

    private final Vertex v1;
    private final Vertex v2;

    Edge(Vertex v1, Vertex v2) {
      this.v1 = v1;
      this.v2 = v2;
    }
  }

  enum Tetrahedron {
    T1(Vertex.V000, Vertex.V001, Vertex.V011, Vertex.V111),
    T2(Vertex.V000, Vertex.V010, Vertex.V011, Vertex.V111),
    T3(Vertex.V000, Vertex.V001, Vertex.V101, Vertex.V111),
    T4(Vertex.V000, Vertex.V100, Vertex.V101, Vertex.V111),
    T5(Vertex.V000, Vertex.V010, Vertex.V110, Vertex.V111),
    T6(Vertex.V000, Vertex.V100, Vertex.V110, Vertex.V111);
    private final Vertex v1;
    private final Vertex v2;
    private final Vertex v3;
    private final Vertex v4;

    Tetrahedron(Vertex v1, Vertex v2, Vertex v3, Vertex v4) {
      this.v1 = v1;
      this.v2 = v2;
      this.v3 = v3;
      this.v4 = v4;
    }
  }

  public enum JointOption {
    INTERNAL,
    SIDES,
    EDGES
  }

  EnumSet<JointOption> jointOptions;
  Map<Vertex, Body> rigidBodies;
  Map<Pair<Vertex, Vertex>, DDoubleBallJoint> vertexToVertexJoints;
  List<Body> ulteriorBodies;
  protected List<Sensor> sensors;
  private final double sideLength;
  private final double mass;
  private final double springConstant;
  private final double dampingConstant;
  private final double rigidMassLengthRatio;
  private final double restVolume;

  public Voxel(
      double sideLength,
      double mass,
      double springConstant,
      double dampingConstant,
      double rigidMassLengthRatio,
      double minVolumeRatio,
      double maxVolumeRatio,
      EnumSet<JointOption> jointOptions,
      String sensorConfig) {
    this.sideLength = sideLength;
    this.mass = mass;
    this.springConstant = springConstant;
    this.dampingConstant = dampingConstant;
    if (rigidMassLengthRatio < 0 || rigidMassLengthRatio >= .5)
      throw new IllegalArgumentException(
          String.format(
              "Attempted to construct voxel with invalid rigid mass length ratio (%.2f)",
              rigidMassLengthRatio));
    this.rigidMassLengthRatio = rigidMassLengthRatio;
    double[] volumeRatio = new double[] {minVolumeRatio > 0 ? minVolumeRatio : 0, maxVolumeRatio};
    this.restVolume = sideLength * sideLength * sideLength;
    this.jointOptions = jointOptions;
    this.rigidBodies = new LinkedHashMap<>(8);
    this.vertexToVertexJoints = new LinkedHashMap<>();
    this.ulteriorBodies = new ArrayList<>();
    this.sensors = new ArrayList<>();
    for (String s : sensorConfig.split("-")) {
      switch (s) {
        case "a":
          sensors.add(new AngleSensor(this));
          break;
        case "v":
          sensors.add(new VolumeRatioSensor(this));
          // TODO ADD SENSORS
      }
    }
  }

  @Override
  public List<Body> bodyParts() {
    return Stream.concat(rigidBodies.values().stream(), ulteriorBodies.stream()).toList();
  }

  @Override
  public List<? extends DJoint> internalJoints() {
    return vertexToVertexJoints.values().stream().toList();
  }

  @Override
  public double restVolume() {
    return restVolume;
  }

  @Override
  public double currentVolume() {
    // TODO DEBUG (CENTER PROBLEM)
    double volume = 0d;
    Map<Vertex, Vector3D> currentVectorsFromV000 = new HashMap<>(7);
    for (Vertex v :
        List.of(
            Vertex.V001,
            Vertex.V010,
            Vertex.V011,
            Vertex.V100,
            Vertex.V101,
            Vertex.V110,
            Vertex.V111)) {
      currentVectorsFromV000.put(
          v, rigidBodies.get(v).position().vectorDistance(rigidBodies.get(Vertex.V000).position()));
    }
    for (Tetrahedron t : Tetrahedron.values()) {
      volume +=
          Math.abs(
              currentVectorsFromV000
                  .get(t.v2)
                  .vectorProduct(currentVectorsFromV000.get(t.v3))
                  .scalarProduct(currentVectorsFromV000.get(t.v4)));
    }
    return volume / 6;
  }

  @Override
  public double[] angle() {
    double[] angle = new double[3];
    Vector3D angleVector1 =
        Stream.of(Side.RIGHT.v1, Side.RIGHT.v2, Side.RIGHT.v3, Side.RIGHT.v4)
            .map(v -> rigidBodies.get(v).position())
            .reduce(Vector3D::sum)
            .get()
            .sum(
                Stream.of(Side.LEFT.v1, Side.LEFT.v2, Side.LEFT.v3, Side.LEFT.v4)
                    .map(v -> rigidBodies.get(v).position())
                    .reduce(Vector3D::sum)
                    .get()
                    .times(-1d));
    Vector3D angleVector2 =
        Stream.of(Side.FRONT.v1, Side.FRONT.v2, Side.FRONT.v3, Side.FRONT.v4)
            .map(v -> rigidBodies.get(v).position())
            .reduce(Vector3D::sum)
            .get()
            .sum(
                Stream.of(Side.BACK.v1, Side.BACK.v2, Side.BACK.v3, Side.BACK.v4)
                    .map(v -> rigidBodies.get(v).position())
                    .reduce(Vector3D::sum)
                    .get()
                    .times(-1d));
    Vector3D angleVector3 =
        Stream.of(Side.UP.v1, Side.UP.v2, Side.UP.v3, Side.UP.v4)
            .map(v -> rigidBodies.get(v).position())
            .reduce(Vector3D::sum)
            .get()
            .sum(
                Stream.of(Side.DOWN.v1, Side.DOWN.v2, Side.DOWN.v3, Side.DOWN.v4)
                    .map(v -> rigidBodies.get(v).position())
                    .reduce(Vector3D::sum)
                    .get()
                    .times(-1d));
    angleVector1 = angleVector1.times(1d / angleVector1.norm());
    angleVector2 = angleVector2.sum(angleVector1.times(-angleVector1.scalarProduct(angleVector2)));
    angleVector2 = angleVector2.times(1d / angleVector2.norm());
    angleVector3 =
        angleVector3.sum(
            angleVector1
                .times(-angleVector1.scalarProduct(angleVector3))
                .sum(angleVector2.times(-angleVector2.scalarProduct(angleVector3))));
    angleVector3 = angleVector3.times(1d / angleVector3.norm());
    angle[0] = Math.atan2(angleVector2.z(), angleVector3.z());
    angle[1] =
        Math.atan2(
            -angleVector1.z(),
            Math.sqrt(angleVector2.z() * angleVector2.z() + angleVector3.z() * angleVector3.z()));
    angle[2] = Math.atan2(angleVector1.y(), angleVector1.x());
    return angle;
  }

  @Override
  public void assemble(Ode4jEngine engine, Vector3D position) {
    double centerShift = sideLength * (1 - rigidMassLengthRatio) / 2d;
    double rigidSphereRadius = sideLength * rigidMassLengthRatio / 2d;
    double rigidSphereMass = mass / 9d;
    // building rigid bodies
    for (Vertex v : Vertex.values()) {
      rigidBodies.put(v, new Sphere(rigidSphereRadius, rigidSphereMass));
    }
    rigidBodies
        .get(Vertex.V000)
        .assemble(
            engine,
            new Vector3D(
                position.x() - centerShift,
                position.y() - centerShift,
                position.z() - centerShift));
    rigidBodies
        .get(Vertex.V001)
        .assemble(
            engine,
            new Vector3D(
                position.x() - centerShift,
                position.y() - centerShift,
                position.z() + centerShift));
    rigidBodies
        .get(Vertex.V010)
        .assemble(
            engine,
            new Vector3D(
                position.x() - centerShift,
                position.y() + centerShift,
                position.z() - centerShift));
    rigidBodies
        .get(Vertex.V011)
        .assemble(
            engine,
            new Vector3D(
                position.x() - centerShift,
                position.y() + centerShift,
                position.z() + centerShift));
    rigidBodies
        .get(Vertex.V100)
        .assemble(
            engine,
            new Vector3D(
                position.x() + centerShift,
                position.y() - centerShift,
                position.z() - centerShift));
    rigidBodies
        .get(Vertex.V101)
        .assemble(
            engine,
            new Vector3D(
                position.x() + centerShift,
                position.y() - centerShift,
                position.z() + centerShift));
    rigidBodies
        .get(Vertex.V110)
        .assemble(
            engine,
            new Vector3D(
                position.x() + centerShift,
                position.y() + centerShift,
                position.z() - centerShift));
    rigidBodies
        .get(Vertex.V111)
        .assemble(
            engine,
            new Vector3D(
                position.x() + centerShift,
                position.y() + centerShift,
                position.z() + centerShift));

    // building joints
    if (jointOptions.contains(JointOption.EDGES)) {
      for (Edge e : Edge.values()) {
        vertexToVertexJoints.put(
            new Pair<>(e.v1, e.v2),
            engine.addSpringJoint(
                rigidBodies.get(e.v1), rigidBodies.get(e.v2), springConstant, dampingConstant));
      }
    }
    if (jointOptions.contains(JointOption.SIDES)) {
      for (Side s : Side.values()) {
        vertexToVertexJoints.put(
            new Pair<>(s.v1, s.v3),
            engine.addSpringJoint(
                rigidBodies.get(s.v1), rigidBodies.get(s.v3), springConstant, dampingConstant));
        vertexToVertexJoints.put(
            new Pair<>(s.v2, s.v4),
            engine.addSpringJoint(
                rigidBodies.get(s.v2), rigidBodies.get(s.v4), springConstant, dampingConstant));
      }
    }
    if (jointOptions.contains(JointOption.INTERNAL)) {
      vertexToVertexJoints.put(
          new Pair<>(Vertex.V000, Vertex.V111),
          engine.addSpringJoint(
              rigidBodies.get(Vertex.V000),
              rigidBodies.get(Vertex.V111),
              springConstant,
              dampingConstant));
      vertexToVertexJoints.put(
          new Pair<>(Vertex.V001, Vertex.V110),
          engine.addSpringJoint(
              rigidBodies.get(Vertex.V001),
              rigidBodies.get(Vertex.V110),
              springConstant,
              dampingConstant));
      vertexToVertexJoints.put(
          new Pair<>(Vertex.V010, Vertex.V101),
          engine.addSpringJoint(
              rigidBodies.get(Vertex.V010),
              rigidBodies.get(Vertex.V101),
              springConstant,
              dampingConstant));
      vertexToVertexJoints.put(
          new Pair<>(Vertex.V011, Vertex.V100),
          engine.addSpringJoint(
              rigidBodies.get(Vertex.V011),
              rigidBodies.get(Vertex.V100),
              springConstant,
              dampingConstant));
    }
  }

  public List<Action> actOnInput() {
    return List.of();
  }
}
