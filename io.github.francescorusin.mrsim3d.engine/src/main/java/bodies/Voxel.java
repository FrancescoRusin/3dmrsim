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

import engine.Ode4jEngine;
import geometry.Vector3D;
import java.util.*;
import java.util.stream.Stream;
import org.ode4j.ode.DDoubleBallJoint;
import org.ode4j.ode.DJoint;
import sensors.AngleSensor;
import sensors.Sensor;
import sensors.VelocitySensor;
import sensors.VolumeRatioSensor;
import test.VisualTest;
import utils.UnorderedPair;

import static drawstuff.DrawStuff.*;

public class Voxel extends MultiBody implements SoftBody {
  //TODO ADD CENTRAL BODY
  private static final double DEFAULT_BODY_CENTER_TO_BODY_CENTER_LENGTH = 0.7;
  private static final double DEFAULT_RIGID_BODY_LENGTH = 0.3;
  private static final double DEFAULT_MASS = 1d;
  private static final double DEFAULT_SPRING_CONSTANT = 300d;
  private static final double DEFAULT_DAMPING_CONSTANT = 300d;
  private static final double DEFAULT_SIDE_LENGTH_STRETCH_RATIO = .2;
  private static final double DEFAULT_CENTER_SPHERE_MASS_RATIO = .5;

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

  public enum Side {
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
    private final List<Edge> edges;

    Side(Vertex v1, Vertex v2, Vertex v3, Vertex v4) {
      this.v1 = v1;
      this.v2 = v2;
      this.v3 = v3;
      this.v4 = v4;
      this.edges = new ArrayList<>();
      List<UnorderedPair<Vertex>> vertexPairs =
              List.of(
                      new UnorderedPair<>(this.v1, this.v2),
                      new UnorderedPair<>(this.v2, this.v3),
                      new UnorderedPair<>(this.v3, this.v4),
                      new UnorderedPair<>(this.v1, this.v4));
      for (Edge e : Edge.values()) {
        UnorderedPair<Vertex> vertexPair = new UnorderedPair<>(e.v1, e.v2);
        if (vertexPairs.contains(vertexPair)) {
          edges.add(e);
        }
      }
    }

    public List<Edge> edges() {
      return edges;
    }

    public List<Vertex> vertices() {
      return List.of(v1, v2, v3, v4);
    }
  }

  public enum Edge {
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

    public final Vertex v1;
    public final Vertex v2;

    Edge(Vertex v1, Vertex v2) {
      this.v1 = v1;
      this.v2 = v2;
    }
  }

  public enum Tetrahedron {
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
    SIDES,
    EDGES_PARALLEL,
    EDGES_CROSSES
  }

  private final EnumSet<JointOption> jointOptions;
  protected EnumMap<Vertex, Body> rigidBodies;
  protected Map<UnorderedPair<Vertex>, List<DDoubleBallJoint>> vertexToVertexJoints;
  protected Map<Vertex, DDoubleBallJoint> centralJoints;
  protected List<Body> ulteriorBodies;
  protected List<Sensor> sensors;
  private final double bodyCenterToBodyCenterLength;
  private final double rigidBodyLength;
  private final double mass;
  private final double centralMassRatio;
  private final double springConstant;
  private final double dampingConstant;
  private final double restVolume;
  private final double[] edgeLengthControlRatio;

  private enum Cache {
    ANGLE, POSITION, VELOCITY, VOLUME
  }

  private final EnumMap<Cache, Double> cacheTime;
  private Vector3D angleCacher;
  private Vector3D positionCacher;
  private Vector3D velocityCacher;
  private double volumeCacher;

  public Voxel(
          double bodyCenterToBodyCenterLength,
          double rigidBodyLength,
          double mass,
          double centralMassRatio,
          double springConstant,
          double dampingConstant,
          double sideLengthStretchRatio,
          EnumSet<JointOption> jointOptions,
          String sensorConfig) {
    this.springConstant = springConstant;
    this.dampingConstant = dampingConstant;
    if (bodyCenterToBodyCenterLength < rigidBodyLength * 2) {
      throw new IllegalArgumentException(
              String.format(
                      "Attempted to construct voxel with invalid rigid sphere length ratio (length %.2f on total %.2f)",
                      rigidBodyLength, bodyCenterToBodyCenterLength));
    }
    this.rigidBodyLength = rigidBodyLength;
    this.bodyCenterToBodyCenterLength = bodyCenterToBodyCenterLength;
    this.mass = mass;
    this.centralMassRatio = centralMassRatio;
    this.edgeLengthControlRatio =
            new double[]{
                    Math.max(1 - sideLengthStretchRatio, 0d), Math.min(1 + sideLengthStretchRatio, 2d)
            };
    this.restVolume =
            this.bodyCenterToBodyCenterLength
                    * this.bodyCenterToBodyCenterLength
                    * this.bodyCenterToBodyCenterLength;
    this.jointOptions = jointOptions;
    this.rigidBodies = new EnumMap<>(Vertex.class);
    this.vertexToVertexJoints = new LinkedHashMap<>();
    this.centralJoints = new LinkedHashMap<>();
    this.sensors = new ArrayList<>();
    this.cacheTime = new EnumMap<>(Cache.class);
    for (String s : sensorConfig.split("-")) {
      switch (s) {
        case "ang":
          sensors.add(new AngleSensor(this));
          break;
        case "vlm":
          sensors.add(new VolumeRatioSensor(this));
          break;
        case "vlc":
          sensors.add(new VelocitySensor(this));
          // TODO ADD SENSORS
      }
    }
  }

  public Voxel(EnumSet<JointOption> jointOptions, String sensorConfig) {
    this(DEFAULT_BODY_CENTER_TO_BODY_CENTER_LENGTH, DEFAULT_RIGID_BODY_LENGTH, DEFAULT_MASS, DEFAULT_CENTER_SPHERE_MASS_RATIO,
            DEFAULT_SPRING_CONSTANT, DEFAULT_DAMPING_CONSTANT, DEFAULT_SIDE_LENGTH_STRETCH_RATIO,
            jointOptions, sensorConfig);
  }

  @Override
  public List<Body> bodyParts() {
    return Stream.concat(rigidBodies.values().stream(), ulteriorBodies.stream()).toList();
  }

  public List<Sensor> sensors() {
    return sensors;
  }

  public Body getVertexBody(Vertex v) {
    return rigidBodies.get(v);
  }

  @Override
  public List<? extends DJoint> internalJoints() {
    return Stream.concat(vertexToVertexJoints.values().stream().flatMap(List::stream),
            centralJoints.values().stream()).toList();
  }

  @Override
  public double baseVolume() {
    return restVolume;
  }

  @Override
  public double minVolume() {
    return restVolume
            * edgeLengthControlRatio[0]
            * edgeLengthControlRatio[0]
            * edgeLengthControlRatio[0];
  }

  @Override
  public double maxVolume() {
    return restVolume
            * edgeLengthControlRatio[1]
            * edgeLengthControlRatio[1]
            * edgeLengthControlRatio[1];
  }

  @Override
  public double currentVolume(double t) {
    if (cacheTime.get(Cache.VOLUME) != t) {
      cacheTime.put(Cache.VOLUME, t);
      double volume = 0d;
      Map<Vertex, Vector3D> currentVectorsFromV000 = new EnumMap<>(Vertex.class);
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
                v, rigidBodies.get(v).position(t).vectorDistance(rigidBodies.get(Vertex.V000).position(t)));
      }
      for (Tetrahedron ttr : Tetrahedron.values()) {
        volume +=
                Math.abs(
                        currentVectorsFromV000
                                .get(ttr.v2)
                                .vectorProduct(currentVectorsFromV000.get(ttr.v3))
                                .scalarProduct(currentVectorsFromV000.get(ttr.v4)));
      }
      volumeCacher = volume / 6d;
    }
    return volumeCacher;
  }

  @Override
  public Vector3D position(double t) {
    if (cacheTime.get(Cache.POSITION) != t) {
      cacheTime.put(Cache.POSITION, t);
      positionCacher = super.position(t);
    }
    return positionCacher;
  }

  @Override
  public Vector3D velocity(double t) {
    if (cacheTime.get(Cache.VELOCITY) != t) {
      cacheTime.put(Cache.VELOCITY, t);
      velocityCacher = super.velocity(t);
    }
    return velocityCacher;
  }

  @Override
  public Vector3D angle(double t) {
    if (cacheTime.get(Cache.ANGLE) != t) {
      cacheTime.put(Cache.ANGLE, t);
      double[] angle = new double[3];
      Vector3D angleVector1 =
              Stream.of(Side.RIGHT.v1, Side.RIGHT.v2, Side.RIGHT.v3, Side.RIGHT.v4)
                      .map(v -> rigidBodies.get(v).position(t))
                      .reduce(Vector3D::sum)
                      .get()
                      .sum(
                              Stream.of(Side.LEFT.v1, Side.LEFT.v2, Side.LEFT.v3, Side.LEFT.v4)
                                      .map(v -> rigidBodies.get(v).position(t))
                                      .reduce(Vector3D::sum)
                                      .get()
                                      .times(-1d));
      Vector3D angleVector2 =
              Stream.of(Side.FRONT.v1, Side.FRONT.v2, Side.FRONT.v3, Side.FRONT.v4)
                      .map(v -> rigidBodies.get(v).position(t))
                      .reduce(Vector3D::sum)
                      .get()
                      .sum(
                              Stream.of(Side.BACK.v1, Side.BACK.v2, Side.BACK.v3, Side.BACK.v4)
                                      .map(v -> rigidBodies.get(v).position(t))
                                      .reduce(Vector3D::sum)
                                      .get()
                                      .times(-1d));
      Vector3D angleVector3 =
              Stream.of(Side.UP.v1, Side.UP.v2, Side.UP.v3, Side.UP.v4)
                      .map(v -> rigidBodies.get(v).position(t))
                      .reduce(Vector3D::sum)
                      .get()
                      .sum(
                              Stream.of(Side.DOWN.v1, Side.DOWN.v2, Side.DOWN.v3, Side.DOWN.v4)
                                      .map(v -> rigidBodies.get(v).position(t))
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
      angle[1] = Math.asin(-angleVector1.z());
      angle[2] = Math.atan2(angleVector1.y(), angleVector1.x());
      angleCacher = new Vector3D(angle[0], angle[1], angle[2]);
    }
    return angleCacher;
  }

  @Override
  public void assemble(Ode4jEngine engine, Vector3D position) {
    for (Cache c : Cache.values()) {
      cacheTime.put(c, -1d);
    }
    double centerShift = bodyCenterToBodyCenterLength / 2d;
    double centerSphereMass = mass * centralMassRatio;
    double rigidSphereMass = (mass - centerSphereMass) / 8d;

    // building rigid bodies
    for (Vertex v : Vertex.values()) {
      rigidBodies.put(v, new Cube(rigidBodyLength, rigidSphereMass));
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
    ulteriorBodies = new ArrayList<>();
    Sphere centralSphere = new Sphere(bodyCenterToBodyCenterLength / 2, centerSphereMass);
    ulteriorBodies.add(centralSphere);
    centralSphere.assemble(engine, position);
    for (Body b : rigidBodies.values()) {
      engine.addCollisionException(centralSphere, b);
    }

    // building joints
    if (jointOptions.contains(JointOption.EDGES_PARALLEL)) {
      for (Edge e : Edge.values()) {
        vertexToVertexJoints.put(new UnorderedPair<>(e.v1, e.v2), new ArrayList<>());
        double squareTick = rigidBodyLength / 2;
        for (double a : List.of(-1d, 1d)) {
          for (double b : List.of(-1d, 1d)) {
            int differentIndex = e.v1.name().charAt(0) == e.v2.name().charAt(0) ? 0 :
                    e.v1.name().charAt(1) == e.v2.name().charAt(1) ? 1 : 2;
            switch (differentIndex) {
              case 0:
                vertexToVertexJoints.get(new UnorderedPair<>(e.v1, e.v2)).add(
                        engine.addSpringJoint(rigidBodies.get(e.v1), rigidBodies.get(e.v2),
                                springConstant, dampingConstant,
                                new Vector3D(squareTick, squareTick * a, squareTick * b),
                                new Vector3D(-squareTick, squareTick * a, squareTick * b)));
                break;
              case 1:
                vertexToVertexJoints.get(new UnorderedPair<>(e.v1, e.v2)).add(
                        engine.addSpringJoint(rigidBodies.get(e.v1), rigidBodies.get(e.v2),
                                springConstant, dampingConstant,
                                new Vector3D(squareTick * a, squareTick, squareTick * b),
                                new Vector3D(squareTick * a, -squareTick, squareTick * b)));
              case 2:
                vertexToVertexJoints.get(new UnorderedPair<>(e.v1, e.v2)).add(
                        engine.addSpringJoint(rigidBodies.get(e.v1), rigidBodies.get(e.v2),
                                springConstant, dampingConstant,
                                new Vector3D(squareTick * a, squareTick * b, squareTick),
                                new Vector3D(squareTick * a, squareTick * b, -squareTick)));
            }
          }
        }
      }
    }
    if (jointOptions.contains(JointOption.EDGES_CROSSES)) {
      for (Edge e : Edge.values()) {
        vertexToVertexJoints.put(new UnorderedPair<>(e.v1, e.v2), new ArrayList<>());
        double squareTick = rigidBodyLength / 2;
        for (double a : List.of(-1d, 1d)) {
          for (double b : List.of(-1d, 1d)) {
            int differentIndex = e.v1.name().charAt(0) == e.v2.name().charAt(0) ? 0 :
                    e.v1.name().charAt(1) == e.v2.name().charAt(1) ? 1 : 2;
            switch (differentIndex) {
              case 0:
                vertexToVertexJoints.get(new UnorderedPair<>(e.v1, e.v2)).add(
                        engine.addSpringJoint(rigidBodies.get(e.v1), rigidBodies.get(e.v2),
                                springConstant, dampingConstant,
                                new Vector3D(squareTick, squareTick * a, squareTick * b),
                                new Vector3D(-squareTick, -squareTick * a, -squareTick * b)));
                break;
              case 1:
                vertexToVertexJoints.get(new UnorderedPair<>(e.v1, e.v2)).add(
                        engine.addSpringJoint(rigidBodies.get(e.v1), rigidBodies.get(e.v2),
                                springConstant, dampingConstant,
                                new Vector3D(squareTick * a, squareTick, squareTick * b),
                                new Vector3D(-squareTick * a, -squareTick, -squareTick * b)));
              case 2:
                vertexToVertexJoints.get(new UnorderedPair<>(e.v1, e.v2)).add(
                        engine.addSpringJoint(rigidBodies.get(e.v1), rigidBodies.get(e.v2),
                                springConstant, dampingConstant,
                                new Vector3D(squareTick * a, squareTick * b, squareTick),
                                new Vector3D(-squareTick * a, -squareTick * b, -squareTick)));
            }
          }
        }
      }
    }
    if (jointOptions.contains(JointOption.SIDES)) {
      for (Side s : Side.values()) {
        vertexToVertexJoints.put(
                new UnorderedPair<>(s.v1, s.v3),
                new ArrayList<>(List.of(engine.addSpringJoint(
                        rigidBodies.get(s.v1), rigidBodies.get(s.v3), springConstant, dampingConstant))));
        vertexToVertexJoints.put(
                new UnorderedPair<>(s.v2, s.v4),
                new ArrayList<>(List.of(engine.addSpringJoint(
                        rigidBodies.get(s.v2), rigidBodies.get(s.v4), springConstant, dampingConstant))));
      }
    }
    for (Vertex v : Vertex.values()) {
      centralJoints.put(v, engine.addSpringJoint(centralSphere, rigidBodies.get(v), springConstant, dampingConstant));
    }
  }

  @Override
  public void rotate(Vector3D eulerAngles) {
    //TODO
  }

  @Override
  public void draw(VisualTest test) {
    for (Body body : bodyParts()) {
      body.draw(test);
    }
    dsSetColor(0, 0, 0);
    dsSetTexture(DS_TEXTURE_NUMBER.DS_WOOD);
    for (DJoint joint : internalJoints()) {
      if (joint instanceof DDoubleBallJoint doubleBallJoint) {
        dsDrawLine(
                doubleBallJoint.getBody(0).getPosition(), doubleBallJoint.getBody(1).getPosition());
      }
    }
  }

  public void actOnInput(EnumMap<Edge, Double> input) {
    // input is assumed to already be in [-1;1]; we have no time to waste on checks
    // apply on edges
    //TODO REFACTOR ACTUATION
    EnumMap<Edge, Double> newLengthRatio = new EnumMap<>(Edge.class);
    for (Edge edge : Edge.values()) {
      newLengthRatio.put(edge, bodyCenterToBodyCenterLength *
              (edgeLengthControlRatio[0] + (edgeLengthControlRatio[1] - edgeLengthControlRatio[0]) *
                      (input.get(edge) / 2 + .5)));
    }

    // apply on edges
    if (jointOptions.contains(JointOption.EDGES)) {
      for (Edge edge : Edge.values()) {
        vertexToVertexJoints
                .get(new UnorderedPair<>(edge.v1, edge.v2))
                .setDistance(newLengthRatio.get(edge));
      }
    }

    // apply on sides
    if (jointOptions.contains(JointOption.SIDES)) {
      for (Side side : Side.values()) {
        final double sideValue = Math.sqrt(side.edges.stream().mapToDouble(newLengthRatio::get).map(d -> d * d).sum() / 2);
        vertexToVertexJoints
                .get(new UnorderedPair<>(side.v1, side.v3))
                .setDistance(sideValue);
        vertexToVertexJoints
                .get(new UnorderedPair<>(side.v2, side.v4))
                .setDistance(sideValue);
      }
    }

    // apply on internal
    EnumMap<Vertex, Double> vertexAverage = new EnumMap<>(Vertex.class);
    for (Vertex v : Vertex.values()) {
      vertexAverage.put(v, 0d);
    }
    for (Edge e : Edge.values()) {
      vertexAverage.put(e.v1, vertexAverage.get(e.v1) + newLengthRatio.get(e) * newLengthRatio.get(e));
      vertexAverage.put(e.v2, vertexAverage.get(e.v2) + newLengthRatio.get(e) * newLengthRatio.get(e));
    }
    for (Vertex v : Vertex.values()) {
      centralJoints.get(v).setDistance(Math.sqrt(vertexAverage.get(v)) / 2);
    }
  }
}