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
import actions.EmitSignal;
import engine.Ode4jEngine;
import geometry.BoundingBox;
import geometry.Vector3D;
import java.util.*;
import java.util.stream.Stream;

import org.ode4j.math.DVector3;
import org.ode4j.ode.DDoubleBallJoint;
import org.ode4j.ode.DFixedJoint;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DRay;
import sensors.*;
import test.VisualTest;
import utils.Pair;
import utils.UnorderedPair;

import static drawstuff.DrawStuff.*;

public class Voxel extends MultiBody implements SoftBody, SignalEmitter, SignalDetector {
  public static final double DEFAULT_SIDE_LENGTH = 1d;
  public static final double DEFAULT_RIGID_BODY_LENGTH = .2;
  public static final double DEFAULT_MASS = 1d;
  protected static final double DEFAULT_SPRING_CONSTANT = 100d;
  protected static final double DEFAULT_DAMPING_CONSTANT = 20d;
  protected static final double DEFAULT_SIDE_LENGTH_STRETCH_RATIO = .2;
  protected static final double DEFAULT_CENTRAL_MASS_RATIO = .01;

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
    DOWN_LEFT(Vertex.V000, Vertex.V010),
    DOWN_FRONT(Vertex.V010, Vertex.V110),
    DOWN_RIGHT(Vertex.V100, Vertex.V110),
    UP_BACK(Vertex.V001, Vertex.V101),
    UP_RIGHT(Vertex.V101, Vertex.V111),
    UP_FRONT(Vertex.V011, Vertex.V111),
    UP_LEFT(Vertex.V001, Vertex.V011),
    SIDE_BL(Vertex.V000, Vertex.V001),
    SIDE_BR(Vertex.V100, Vertex.V101),
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

  protected enum UlteriorBody {
    CENTRAL_MASS
  }

  private final EnumSet<JointOption> jointOptions;
  protected EnumMap<Vertex, Body> rigidBodies;
  protected Map<UnorderedPair<Vertex>, List<DDoubleBallJoint>> vertexToVertexJoints;
  protected Map<Pair<Vertex, UlteriorBody>, DDoubleBallJoint> ulteriorJoints;
  protected Map<DDoubleBallJoint, Double> jointMaxLength;
  protected Map<DDoubleBallJoint, Double> jointMinLength;
  protected Map<UlteriorBody, Body> ulteriorBodies;
  protected final List<Sensor> internalSensors;
  protected final List<NearFieldSignalSensor> commSensors;
  private final double bodyCenterToBodyCenterLength;
  private final double rigidBodyLength;
  private final double mass;
  private final double centralMassRatio;
  private final double springConstant;
  private final double dampingConstant;
  private final double restVolume;
  private final double[] edgeLengthControlRatio;

  private enum Cache {
    ANGLE, BBOX, POSITION, VELOCITY, VOLUME
  }

  private final EnumMap<Cache, Double> cacheTime;
  private Vector3D angleCacher;
  private Vector3D positionCacher;
  private Vector3D velocityCacher;
  private BoundingBox bBoxCacher;
  private double volumeCacher;

  public Voxel(
          double sideLength,
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
    if (sideLength < rigidBodyLength * 2) {
      throw new IllegalArgumentException(
              String.format(
                      "Attempted to construct voxel with invalid rigid sphere length ratio (length %.2f on total %.2f)",
                      rigidBodyLength, sideLength));
    }
    this.bodyCenterToBodyCenterLength = sideLength - rigidBodyLength;
    this.rigidBodyLength = rigidBodyLength;
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
    this.internalSensors = new ArrayList<>();
    this.commSensors = new ArrayList<>();
    this.cacheTime = new EnumMap<>(Cache.class);
    for (String s : sensorConfig.split("-")) {
      switch (s) {
        case "ang" -> internalSensors.add(new AngleSensor(this));
        case "vlm" -> internalSensors.add(new VolumeRatioSensor(this));
        case "vlc" -> internalSensors.add(new VelocitySensor(this));
        case "scr" -> internalSensors.add(new SideCompressionSensor(this));
        // TODO ADD SENSORS
      }
      if (s.matches("nfs[0-9]")) {
        commSensors.add(new NearFieldSignalSensor(this, Character.getNumericValue(s.charAt(3))));
      }
    }
  }

  public Voxel(EnumSet<JointOption> jointOptions, String sensorConfig) {
    this(DEFAULT_SIDE_LENGTH, DEFAULT_RIGID_BODY_LENGTH, DEFAULT_MASS, DEFAULT_CENTRAL_MASS_RATIO,
            DEFAULT_SPRING_CONSTANT, DEFAULT_DAMPING_CONSTANT, DEFAULT_SIDE_LENGTH_STRETCH_RATIO,
            jointOptions, sensorConfig);
  }

  @Override
  public List<Body> bodyParts() {
    return Stream.concat(rigidBodies.values().stream(), ulteriorBodies.values().stream()).toList();
  }

  @Override
  public List<Body> detectorBodies() {
    return List.of(ulteriorBodies.get(UlteriorBody.CENTRAL_MASS));
  }

  @Override
  public int nOfSides() {
    return 6;
  }

  public List<Sensor> sensors() {
    return Stream.concat(internalSensors.stream(), commSensors.stream()).toList();
  }

  public List<NearFieldSignalSensor> commSensors() {
    return commSensors;
  }

  public Body vertexBody(Vertex v) {
    return rigidBodies.get(v);
  }

  @Override
  public List<? extends DJoint> internalJoints() {
    return Stream.concat(vertexToVertexJoints.values().stream().flatMap(List::stream),
            ulteriorJoints.values().stream()).toList();
  }

  public double bodyCenterToBodyCenterLength() {
    return bodyCenterToBodyCenterLength;
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
  public BoundingBox boundingBox(double t) {
    if (cacheTime.get(Cache.BBOX) != t) {
      cacheTime.put(Cache.BBOX, t);
      bBoxCacher = super.boundingBox(t);
    }
    return bBoxCacher;
  }

  @Override
  public void assemble(Ode4jEngine engine, Vector3D position) {
    rigidBodies = new EnumMap<>(Vertex.class);
    ulteriorBodies = new EnumMap<>(UlteriorBody.class);
    vertexToVertexJoints = new LinkedHashMap<>();
    ulteriorJoints = new LinkedHashMap<>();
    jointMaxLength = new LinkedHashMap<>();
    jointMinLength = new LinkedHashMap<>();
    for (Cache c : Cache.values()) {
      cacheTime.put(c, -1d);
    }
    final double vertexBodyCenterShift = bodyCenterToBodyCenterLength / 2d;
    final double centralSphereMass = centralMassRatio * mass;
    final double rigidSphereMass = (mass - centralSphereMass) / 8d;

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
    final Cube centralCube = new Cube(
            (bodyCenterToBodyCenterLength - rigidBodyLength) * edgeLengthControlRatio[0] + rigidBodyLength,
            centralSphereMass);
    ulteriorBodies.put(UlteriorBody.CENTRAL_MASS, centralCube);
    centralCube.assemble(engine, position);

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
      maxLength = (rigidBodyLength + (bodyCenterToBodyCenterLength - rigidBodyLength) * edgeLengthControlRatio[1]) * Math.sqrt(2);
      minLength = (rigidBodyLength + (bodyCenterToBodyCenterLength - rigidBodyLength) * edgeLengthControlRatio[0]) * Math.sqrt(2);
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
      maxLength = (rigidBodyLength + (bodyCenterToBodyCenterLength - rigidBodyLength) * edgeLengthControlRatio[1]) * Math.sqrt(3);
      minLength = (rigidBodyLength + (bodyCenterToBodyCenterLength - rigidBodyLength) * edgeLengthControlRatio[0]) * Math.sqrt(3);
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
    // central mass joints
    minLength = 0d;
    maxLength = (edgeLengthControlRatio[1] - edgeLengthControlRatio[0]) *
            (bodyCenterToBodyCenterLength - rigidBodyLength) * Math.sqrt(3) / 2;
    for (Vertex v : Vertex.values()) {
      final String vertexName = v.name();
      joint = engine.addSpringJoint(ulteriorBodies.get(UlteriorBody.CENTRAL_MASS), rigidBodies.get(v),
              springConstant, dampingConstant,
              new Vector3D(new double[]{
                      centralCube.sideLength() * (vertexName.charAt(1) == '0' ? -.5 : .5),
                      centralCube.sideLength() * (vertexName.charAt(2) == '0' ? -.5 : .5),
                      centralCube.sideLength() * (vertexName.charAt(3) == '0' ? -.5 : .5)
              }),
              new Vector3D());
      ulteriorJoints.put(new Pair<>(v, UlteriorBody.CENTRAL_MASS), joint);
      jointMinLength.put(joint, minLength);
      jointMaxLength.put(joint, maxLength);
      engine.addCollisionException(rigidBodies.get(v).collisionGeometry(), centralCube.collisionGeometry());
    }
  }

  @Override
  public void rotate(Ode4jEngine engine, Vector3D eulerAngles) {
    super.rotate(engine, eulerAngles);
    cacheTime.put(Cache.ANGLE, -1d);
    cacheTime.put(Cache.BBOX, -1d);
    cacheTime.put(Cache.POSITION, -1d);
    cacheTime.put(Cache.VELOCITY, -1d);
  }

  @Override
  public void translate(Ode4jEngine engine, Vector3D translation) {
    super.translate(engine, translation);
    cacheTime.put(Cache.BBOX, -1d);
    cacheTime.put(Cache.POSITION, -1d);
  }

  @Override
  public void draw(VisualTest test) {
    dsSetColor(0, 0, 1);
    dsSetTexture(DS_TEXTURE_NUMBER.DS_WOOD);
    rigidBodies.values().forEach(body -> body.draw(test));
    dsSetColor(1, 0, 0);
    ulteriorBodies.values().forEach(body -> body.draw(test));
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
    EnumMap<Edge, Double> denormalizedInput = new EnumMap<>(Edge.class);
    for (Edge edge : Edge.values()) {
      denormalizedInput.put(edge, Math.max(Math.min(input.get(edge) * .5 + .5, 1d), -1d));
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
    EnumMap<Vertex, Double> vertexSum = new EnumMap<>(Vertex.class);
    for (Vertex v : Vertex.values()) {
      vertexSum.put(v, 0d);
    }
    for (Edge e : Edge.values()) {
      vertexSum.put(e.v1, vertexSum.get(e.v1) + denormalizedInput.get(e));
      vertexSum.put(e.v2, vertexSum.get(e.v2) + denormalizedInput.get(e));
    }

    // apply on internal
    if (jointOptions.contains(JointOption.INTERNAL)) {
      for (DDoubleBallJoint joint : vertexToVertexJoints.get(new UnorderedPair<>(Vertex.V000, Vertex.V111))) {
        joint.setDistance(jointMinLength.get(joint) + (vertexSum.get(Vertex.V000) + vertexSum.get(Vertex.V111)) *
                (jointMaxLength.get(joint) - jointMinLength.get(joint)) / 6);
      }
      for (DDoubleBallJoint joint : vertexToVertexJoints.get(new UnorderedPair<>(Vertex.V001, Vertex.V110))) {
        joint.setDistance(jointMinLength.get(joint) + (vertexSum.get(Vertex.V001) + vertexSum.get(Vertex.V110)) *
                (jointMaxLength.get(joint) - jointMinLength.get(joint)) / 6);
      }
      for (DDoubleBallJoint joint : vertexToVertexJoints.get(new UnorderedPair<>(Vertex.V010, Vertex.V101))) {
        joint.setDistance(jointMinLength.get(joint) + (vertexSum.get(Vertex.V010) + vertexSum.get(Vertex.V101)) *
                (jointMaxLength.get(joint) - jointMinLength.get(joint)) / 6);
      }
      for (DDoubleBallJoint joint : vertexToVertexJoints.get(new UnorderedPair<>(Vertex.V011, Vertex.V100))) {
        joint.setDistance(jointMinLength.get(joint) + (vertexSum.get(Vertex.V011) + vertexSum.get(Vertex.V100)) *
                (jointMaxLength.get(joint) - jointMinLength.get(joint)) / 6);
      }
    }

    // apply on central body
    for (Vertex v : Vertex.values()) {
      DDoubleBallJoint joint = ulteriorJoints.get(new Pair<>(v, UlteriorBody.CENTRAL_MASS));
      joint.setDistance(jointMinLength.get(joint) +
              vertexSum.get(v) * (jointMaxLength.get(joint) - jointMinLength.get(joint)) / 3);
    }
  }

  @Override
  public List<Action> emitSignals(Ode4jEngine engine, double length, int channel, double[] values) {
    double shift = bodyCenterToBodyCenterLength / 2 + length;
    int index = -1;
    List<Action> outputActions = new ArrayList<>();
    for (Vector3D sideCenter : List.of(
            new Vector3D(0, 0, 1), new Vector3D(0, 0, -1),
            new Vector3D(0, 1, 0), new Vector3D(0, -1, 0),
            new Vector3D(1, 0, 0), new Vector3D(-1, 0, 0))) {
      outputActions.add(new EmitSignal(this, sideCenter.rotate(angle(engine.t())),
              shift, channel, values[++index]));
    }
    return outputActions;
  }

  @Override
  public void readSignal(Ode4jEngine engine, DRay signal, Vector3D contactPosition) {
    if (commSensors.isEmpty()) {
      return;
    }
    Vector3D relativeContactPosition = contactPosition.vectorDistance(position(engine.t())).reverseRotate(angle(engine.t()));
    Side side = closestSide(relativeContactPosition);
    int channel = Math.toIntExact(~signal.getCollideBits());
    for (NearFieldSignalSensor sensor : commSensors) {
      if (sensor.channel == channel) {
        sensor.readSignal(Double.longBitsToDouble(signal.getCategoryBits()), side);
      }
    }
  }

  private static Side closestSide(Vector3D relativeContactPosition) {
    Vector3D absoluteValues = relativeContactPosition.forEach(Math::abs);
    Side side;
    if (absoluteValues.x() > absoluteValues.y() && absoluteValues.x() > absoluteValues.z()) {
      if (relativeContactPosition.x() > 0) {
        side = Side.RIGHT;
      } else {
        side = Side.LEFT;
      }
    } else if (absoluteValues.y() > absoluteValues.x() && absoluteValues.y() > absoluteValues.z()) {
      if (relativeContactPosition.y() > 0) {
        side = Side.FRONT;
      } else {
        side = Side.BACK;
      }
    } else {
      if (relativeContactPosition.z() > 0) {
        side = Side.UP;
      } else {
        side = Side.DOWN;
      }
    }
    return side;
  }
}