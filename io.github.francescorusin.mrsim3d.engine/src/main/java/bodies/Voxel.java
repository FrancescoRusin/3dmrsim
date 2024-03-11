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

import org.ode4j.math.DVector3;
import org.ode4j.ode.DDoubleBallJoint;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.OdeHelper;
import sensors.AngleSensor;
import sensors.Sensor;
import sensors.VelocitySensor;
import sensors.VolumeRatioSensor;
import test.VisualTest;
import utils.UnorderedPair;

import static drawstuff.DrawStuff.*;

public class Voxel extends MultiBody implements SoftBody {
  private static final double DEFAULT_BODY_CENTER_TO_BODY_CENTER_LENGTH = 0.8;
  private static final double DEFAULT_RIGID_BODY_LENGTH = 0.2;
  private static final double DEFAULT_MASS = 1d;
  private static final double DEFAULT_SPRING_CONSTANT = 100d;
  private static final double DEFAULT_DAMPING_CONSTANT = 20d;
  private static final double DEFAULT_SIDE_LENGTH_STRETCH_RATIO = .2;
  private static final double DEFAULT_CENTER_SPHERE_MASS_RATIO = 0d;

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
    public final Vertex v1;
    public final Vertex v2;
    public final Vertex v3;
    public final Vertex v4;
    public final List<Edge> edges;

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
        if (vertexPairs.contains(new UnorderedPair<>(e.v1, e.v2))) {
          edges.add(e);
        }
      }
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
    public final Vertex v1;
    public final Vertex v2;
    public final Vertex v3;
    public final Vertex v4;

    Tetrahedron(Vertex v1, Vertex v2, Vertex v3, Vertex v4) {
      this.v1 = v1;
      this.v2 = v2;
      this.v3 = v3;
      this.v4 = v4;
    }
  }

  public enum JointOption {
    EDGES_PARALLEL,
    EDGES_CROSSES,
    EDGES_DIAGONALS,
    SIDES,
    INTERNAL
  }

  private final EnumSet<JointOption> jointOptions;
  protected EnumMap<Vertex, Body> rigidBodies;
  protected Map<UnorderedPair<Vertex>, List<DDoubleBallJoint>> vertexToVertexJoints;
  protected Map<Vertex, DDoubleBallJoint> centralJoints;
  protected Map<DDoubleBallJoint, Double> jointMaxLength;
  protected Map<DDoubleBallJoint, Double> jointMinLength;
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
    this.sensors = new ArrayList<>();
    this.cacheTime = new EnumMap<>(Cache.class);
    for (String s : sensorConfig.split("-")) {
      switch (s) {
        case "ang" -> sensors.add(new AngleSensor(this));
        case "vlm" -> sensors.add(new VolumeRatioSensor(this));
        case "vlc" -> sensors.add(new VelocitySensor(this));
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
    rigidBodies = new EnumMap<>(Vertex.class);
    vertexToVertexJoints = new LinkedHashMap<>();
    centralJoints = new LinkedHashMap<>();
    jointMaxLength = new LinkedHashMap<>();
    jointMinLength = new LinkedHashMap<>();
    for (Cache c : Cache.values()) {
      cacheTime.put(c, -1d);
    }
    double vertexBodyCenterShift = bodyCenterToBodyCenterLength / 2d;
    double centralSphereMass = centralMassRatio * mass;
    double rigidSphereMass = (mass - centralSphereMass) / 8d;

    // building rigid bodies
    for (Vertex v : Vertex.values()) {
      rigidBodies.put(v, new Cube(rigidBodyLength, rigidSphereMass));
    }
    rigidBodies
            .get(Vertex.V000)
            .assemble(
                    engine,
                    new Vector3D(
                            position.x() - vertexBodyCenterShift,
                            position.y() - vertexBodyCenterShift,
                            position.z() - vertexBodyCenterShift));
    rigidBodies
            .get(Vertex.V001)
            .assemble(
                    engine,
                    new Vector3D(
                            position.x() - vertexBodyCenterShift,
                            position.y() - vertexBodyCenterShift,
                            position.z() + vertexBodyCenterShift));
    rigidBodies
            .get(Vertex.V010)
            .assemble(
                    engine,
                    new Vector3D(
                            position.x() - vertexBodyCenterShift,
                            position.y() + vertexBodyCenterShift,
                            position.z() - vertexBodyCenterShift));
    rigidBodies
            .get(Vertex.V011)
            .assemble(
                    engine,
                    new Vector3D(
                            position.x() - vertexBodyCenterShift,
                            position.y() + vertexBodyCenterShift,
                            position.z() + vertexBodyCenterShift));
    rigidBodies
            .get(Vertex.V100)
            .assemble(
                    engine,
                    new Vector3D(
                            position.x() + vertexBodyCenterShift,
                            position.y() - vertexBodyCenterShift,
                            position.z() - vertexBodyCenterShift));
    rigidBodies
            .get(Vertex.V101)
            .assemble(
                    engine,
                    new Vector3D(
                            position.x() + vertexBodyCenterShift,
                            position.y() - vertexBodyCenterShift,
                            position.z() + vertexBodyCenterShift));
    rigidBodies
            .get(Vertex.V110)
            .assemble(
                    engine,
                    new Vector3D(
                            position.x() + vertexBodyCenterShift,
                            position.y() + vertexBodyCenterShift,
                            position.z() - vertexBodyCenterShift));
    rigidBodies
            .get(Vertex.V111)
            .assemble(
                    engine,
                    new Vector3D(
                            position.x() + vertexBodyCenterShift,
                            position.y() + vertexBodyCenterShift,
                            position.z() + vertexBodyCenterShift));
    ulteriorBodies = new ArrayList<>();

    // building joints
    for (Vertex v1 : Vertex.values()) {
      for (Vertex v2 : Vertex.values()) {
        if (v1 != v2) {
          vertexToVertexJoints.put(new UnorderedPair<>(v1, v2), new ArrayList<>());
        }
      }
    }
    double squareTick = rigidBodyLength / 2;
    double maxLength, minLength;
    DDoubleBallJoint joint;
    if (jointOptions.contains(JointOption.EDGES_PARALLEL)) {
      maxLength = (bodyCenterToBodyCenterLength - rigidBodyLength) * edgeLengthControlRatio[1];
      minLength = (bodyCenterToBodyCenterLength - rigidBodyLength) * edgeLengthControlRatio[0];
      for (Edge e : Edge.values()) {
        int differentIndex = e.v1.name().charAt(1) != e.v2.name().charAt(1) ? 0 :
                e.v1.name().charAt(2) != e.v2.name().charAt(2) ? 1 : 2;
        for (double a : List.of(-1d, 1d)) {
          for (double b : List.of(-1d, 1d)) {
            joint = switch (differentIndex) {
              case 0 -> engine.addSpringJoint(rigidBodies.get(e.v1), rigidBodies.get(e.v2),
                      springConstant, dampingConstant,
                      new Vector3D(squareTick, squareTick * a, squareTick * b),
                      new Vector3D(-squareTick, squareTick * a, squareTick * b));
              case 1 -> engine.addSpringJoint(rigidBodies.get(e.v1), rigidBodies.get(e.v2),
                      springConstant, dampingConstant,
                      new Vector3D(squareTick * a, squareTick, squareTick * b),
                      new Vector3D(squareTick * a, -squareTick, squareTick * b));
              case 2 -> engine.addSpringJoint(rigidBodies.get(e.v1), rigidBodies.get(e.v2),
                      springConstant, dampingConstant,
                      new Vector3D(squareTick * a, squareTick * b, squareTick),
                      new Vector3D(squareTick * a, squareTick * b, -squareTick));
              default -> null;
            };
            vertexToVertexJoints.get(new UnorderedPair<>(e.v1, e.v2)).add(joint);
            jointMaxLength.put(joint, maxLength);
            jointMinLength.put(joint, minLength);
          }
        }
      }
    }
    if (jointOptions.contains(JointOption.EDGES_CROSSES)) {
      Vector3D firstPosition;
      maxLength = Math.sqrt(Math.pow((bodyCenterToBodyCenterLength - rigidBodyLength) * edgeLengthControlRatio[1], 2) + Math.pow(rigidBodyLength, 2));
      minLength = Math.sqrt(Math.pow((bodyCenterToBodyCenterLength - rigidBodyLength) * edgeLengthControlRatio[0], 2) + Math.pow(rigidBodyLength, 2));
      for (Edge e : Edge.values()) {
        int differentIndex = e.v1.name().charAt(1) != e.v2.name().charAt(1) ? 0 :
                e.v1.name().charAt(2) != e.v2.name().charAt(2) ? 1 : 2;
        for (double a : List.of(-1d, 1d)) {
          for (double b : List.of(-1d, 1d)) {
            switch (differentIndex) {
              case 0 -> {
                firstPosition = new Vector3D(squareTick, squareTick * a, squareTick * b);
                joint = engine.addSpringJoint(rigidBodies.get(e.v1), rigidBodies.get(e.v2), springConstant, dampingConstant,
                        firstPosition,
                        new Vector3D(-squareTick, squareTick * a, -squareTick * b));
                vertexToVertexJoints.get(new UnorderedPair<>(e.v1, e.v2)).add(joint);
                jointMaxLength.put(joint, maxLength);
                jointMinLength.put(joint, minLength);
                joint = engine.addSpringJoint(rigidBodies.get(e.v1), rigidBodies.get(e.v2), springConstant, dampingConstant,
                        firstPosition,
                        new Vector3D(-squareTick, -squareTick * a, squareTick * b));
                vertexToVertexJoints.get(new UnorderedPair<>(e.v1, e.v2)).add(joint);
                jointMaxLength.put(joint, maxLength);
                jointMinLength.put(joint, minLength);
              }
              case 1 -> {
                firstPosition = new Vector3D(squareTick * a, squareTick, squareTick * b);
                joint = engine.addSpringJoint(rigidBodies.get(e.v1), rigidBodies.get(e.v2), springConstant, dampingConstant,
                        firstPosition,
                        new Vector3D(squareTick * a, -squareTick, -squareTick * b));
                vertexToVertexJoints.get(new UnorderedPair<>(e.v1, e.v2)).add(joint);
                jointMaxLength.put(joint, maxLength);
                jointMinLength.put(joint, minLength);
                joint = engine.addSpringJoint(rigidBodies.get(e.v1), rigidBodies.get(e.v2), springConstant, dampingConstant,
                        firstPosition,
                        new Vector3D(-squareTick * a, -squareTick, squareTick * b));
                vertexToVertexJoints.get(new UnorderedPair<>(e.v1, e.v2)).add(joint);
                jointMaxLength.put(joint, maxLength);
                jointMinLength.put(joint, minLength);
              }
              case 2 -> {
                firstPosition = new Vector3D(squareTick * a, squareTick * b, squareTick);
                joint = engine.addSpringJoint(rigidBodies.get(e.v1), rigidBodies.get(e.v2), springConstant, dampingConstant,
                        firstPosition,
                        new Vector3D(squareTick * a, -squareTick * b, -squareTick));
                vertexToVertexJoints.get(new UnorderedPair<>(e.v1, e.v2)).add(joint);
                jointMaxLength.put(joint, maxLength);
                jointMinLength.put(joint, minLength);
                joint = engine.addSpringJoint(rigidBodies.get(e.v1), rigidBodies.get(e.v2), springConstant, dampingConstant,
                        firstPosition,
                        new Vector3D(-squareTick * a, squareTick * b, -squareTick));
                vertexToVertexJoints.get(new UnorderedPair<>(e.v1, e.v2)).add(joint);
                jointMaxLength.put(joint, maxLength);
                jointMinLength.put(joint, minLength);
              }
            }
          }
        }
      }
    }
    if (jointOptions.contains(JointOption.EDGES_DIAGONALS)) {
      maxLength = Math.sqrt(Math.pow((bodyCenterToBodyCenterLength - rigidBodyLength) * edgeLengthControlRatio[1], 2) + 2 * Math.pow(rigidBodyLength, 2));
      minLength = Math.sqrt(Math.pow((bodyCenterToBodyCenterLength - rigidBodyLength) * edgeLengthControlRatio[0], 2) + 2 * Math.pow(rigidBodyLength, 2));
      for (Edge e : Edge.values()) {
        int differentIndex = e.v1.name().charAt(1) != e.v2.name().charAt(1) ? 0 :
                e.v1.name().charAt(2) != e.v2.name().charAt(2) ? 1 : 2;
        for (double a : List.of(-1d, 1d)) {
          for (double b : List.of(-1d, 1d)) {
            joint = switch (differentIndex) {
              case 0 -> engine.addSpringJoint(rigidBodies.get(e.v1), rigidBodies.get(e.v2),
                      springConstant, dampingConstant,
                      new Vector3D(squareTick, squareTick * a, squareTick * b),
                      new Vector3D(-squareTick, -squareTick * a, -squareTick * b));
              case 1 -> engine.addSpringJoint(rigidBodies.get(e.v1), rigidBodies.get(e.v2),
                      springConstant, dampingConstant,
                      new Vector3D(squareTick * a, squareTick, squareTick * b),
                      new Vector3D(-squareTick * a, -squareTick, -squareTick * b));
              case 2 -> engine.addSpringJoint(rigidBodies.get(e.v1), rigidBodies.get(e.v2),
                      springConstant, dampingConstant,
                      new Vector3D(squareTick * a, squareTick * b, squareTick),
                      new Vector3D(-squareTick * a, -squareTick * b, -squareTick));
              default -> null;
            };
            vertexToVertexJoints.get(new UnorderedPair<>(e.v1, e.v2)).add(joint);
            jointMaxLength.put(joint, maxLength);
            jointMinLength.put(joint, minLength);
          }
        }
      }
    }
    if (jointOptions.contains(JointOption.SIDES)) {
      maxLength = bodyCenterToBodyCenterLength * edgeLengthControlRatio[1] * Math.sqrt(2);
      minLength = bodyCenterToBodyCenterLength * edgeLengthControlRatio[0] * Math.sqrt(2);
      for (Side s : Side.values()) {
        joint = engine.addSpringJoint(rigidBodies.get(s.v1), rigidBodies.get(s.v3), springConstant, dampingConstant);
        vertexToVertexJoints.get(new UnorderedPair<>(s.v1, s.v3)).add(joint);
        jointMaxLength.put(joint, maxLength);
        jointMinLength.put(joint, minLength);
        joint = engine.addSpringJoint(rigidBodies.get(s.v2), rigidBodies.get(s.v4), springConstant, dampingConstant);
        vertexToVertexJoints.get(new UnorderedPair<>(s.v2, s.v4)).add(joint);
        jointMaxLength.put(joint, maxLength);
        jointMinLength.put(joint, minLength);
      }
    }
    if (jointOptions.contains(JointOption.INTERNAL)) {
      maxLength = bodyCenterToBodyCenterLength * edgeLengthControlRatio[1] * Math.sqrt(3);
      minLength = bodyCenterToBodyCenterLength * edgeLengthControlRatio[0] * Math.sqrt(3);
      joint = engine.addSpringJoint(rigidBodies.get(Vertex.V000), rigidBodies.get(Vertex.V111), springConstant, dampingConstant);
      vertexToVertexJoints.get(new UnorderedPair<>(Vertex.V000, Vertex.V111)).add(joint);
      jointMaxLength.put(joint, maxLength);
      jointMinLength.put(joint, minLength);
      joint = engine.addSpringJoint(rigidBodies.get(Vertex.V001), rigidBodies.get(Vertex.V110), springConstant, dampingConstant);
      vertexToVertexJoints.get(new UnorderedPair<>(Vertex.V001, Vertex.V110)).add(joint);
      jointMaxLength.put(joint, maxLength);
      jointMinLength.put(joint, minLength);
      joint = engine.addSpringJoint(rigidBodies.get(Vertex.V010), rigidBodies.get(Vertex.V101), springConstant, dampingConstant);
      vertexToVertexJoints.get(new UnorderedPair<>(Vertex.V010, Vertex.V101)).add(joint);
      jointMaxLength.put(joint, maxLength);
      jointMinLength.put(joint, minLength);
      joint = engine.addSpringJoint(rigidBodies.get(Vertex.V011), rigidBodies.get(Vertex.V100), springConstant, dampingConstant);
      vertexToVertexJoints.get(new UnorderedPair<>(Vertex.V011, Vertex.V100)).add(joint);
      jointMaxLength.put(joint, maxLength);
      jointMinLength.put(joint, minLength);
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
    DVector3 anchor1 = new DVector3();
    DVector3 anchor2 = new DVector3();
    dsSetColor(0, 0, 0);
    dsSetTexture(DS_TEXTURE_NUMBER.DS_WOOD);
    for (DJoint joint : internalJoints()) {
      if (joint instanceof DDoubleBallJoint doubleBallJoint) {
        doubleBallJoint.getAnchor1(anchor1);
        doubleBallJoint.getAnchor2(anchor2);
        dsDrawLine(anchor1, anchor2);
      }
    }
  }

  public void actOnInput(EnumMap<Edge, Double> input) {
    // input is assumed to already be in [-1;1]; we have no time to waste on checks
    //TODO REFACTOR ACTUATION

    EnumMap<Edge, Double> denormalizedInput = new EnumMap<>(Edge.class);
    for (Edge edge : Edge.values()) {
      denormalizedInput.put(edge, (input.get(edge) * .5 + .5));
    }

    // apply on edges
    for (Edge edge : Edge.values()) {
      for (DDoubleBallJoint joint : vertexToVertexJoints.get(new UnorderedPair<>(edge.v1, edge.v2))) {
        joint.setDistance(jointMinLength.get(joint) + denormalizedInput.get(edge) * (jointMaxLength.get(joint) - jointMinLength.get(joint)));
      }
    }

    // apply on sides
    if (jointOptions.contains(JointOption.SIDES)) {
      for (Side side : Side.values()) {
        final double sideValue = side.edges.stream().mapToDouble(denormalizedInput::get).average().orElse(0d);
        for (DDoubleBallJoint joint : vertexToVertexJoints.get(new UnorderedPair<>(side.v1, side.v3))) {
          joint.setDistance(jointMinLength.get(joint) + sideValue * (jointMaxLength.get(joint) - jointMinLength.get(joint)));
        }
        for (DDoubleBallJoint joint : vertexToVertexJoints.get(new UnorderedPair<>(side.v2, side.v4))) {
          joint.setDistance(jointMinLength.get(joint) + sideValue * (jointMaxLength.get(joint) - jointMinLength.get(joint)));
        }
      }
    }

    // apply on internal
    if (jointOptions.contains(JointOption.INTERNAL)) {
      EnumMap<Vertex, Double> vertexAverage = new EnumMap<>(Vertex.class);
      for (Vertex v : Vertex.values()) {
        vertexAverage.put(v, 0d);
      }
      for (Edge e : Edge.values()) {
        vertexAverage.put(e.v1, vertexAverage.get(e.v1) + denormalizedInput.get(e));
        vertexAverage.put(e.v2, vertexAverage.get(e.v2) + denormalizedInput.get(e));
      }
      for (Vertex v : Vertex.values()) {
        vertexAverage.put(v, vertexAverage.get(v));
      }
      for (DDoubleBallJoint joint : vertexToVertexJoints.get(new UnorderedPair<>(Vertex.V000, Vertex.V111))) {
        joint.setDistance(jointMinLength.get(joint) + (vertexAverage.get(Vertex.V000) + vertexAverage.get(Vertex.V111)) *
                (jointMaxLength.get(joint) - jointMinLength.get(joint)) / 6);
      }
      for (DDoubleBallJoint joint : vertexToVertexJoints.get(new UnorderedPair<>(Vertex.V001, Vertex.V110))) {
        joint.setDistance(jointMinLength.get(joint) + (vertexAverage.get(Vertex.V001) + vertexAverage.get(Vertex.V110)) *
                (jointMaxLength.get(joint) - jointMinLength.get(joint)) / 6);
      }
      for (DDoubleBallJoint joint : vertexToVertexJoints.get(new UnorderedPair<>(Vertex.V010, Vertex.V101))) {
        joint.setDistance(jointMinLength.get(joint) + (vertexAverage.get(Vertex.V010) + vertexAverage.get(Vertex.V101)) *
                (jointMaxLength.get(joint) - jointMinLength.get(joint)) / 6);
      }
      for (DDoubleBallJoint joint : vertexToVertexJoints.get(new UnorderedPair<>(Vertex.V011, Vertex.V100))) {
        joint.setDistance(jointMinLength.get(joint) + (vertexAverage.get(Vertex.V011) + vertexAverage.get(Vertex.V100)) *
                (jointMaxLength.get(joint) - jointMinLength.get(joint)) / 6);
      }
    }
  }
}