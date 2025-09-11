package engine; /*-
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
import ad.Attachable;
import agents.EmbodiedAgent;
import bodies.*;
import geometry.Vector3D;
import java.util.*;
import java.util.function.Function;
import java.util.stream.Stream;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;
import sensors.ContactSensor;
import sensors.Sensor;
import snapshot.InstantSnapshot;
import utils.UnorderedPair;

public class Ode4jEngine {
  public record Configuration(
      Vector3D gravity,
      Function<DSpace, DGeom> terrainSupplier,
      double maxAttachDistance,
      double maxAttractDistance,
      double attachSpringRestDistance,
      double attractForceModule,
      double attachSpringConstant,
      double attachDampingConstant,
      double nfcRange) {}

  public static final Configuration DEFAULT_CONFIGURATION =
      new Configuration(
          new Vector3D(0d, 0d, -9.81),
          dSpace -> OdeHelper.createPlane(dSpace, 0, 0, 1, 0),
          Voxel.DEFAULT_SIDE_LENGTH * .3,
          Voxel.DEFAULT_SIDE_LENGTH * 2,
          Voxel.DEFAULT_RIGID_BODY_LENGTH * 1.01,
          8d,
          Voxel.DEFAULT_SPRING_CONSTANT * 10,
          Voxel.DEFAULT_DAMPING_CONSTANT * 10,
          Voxel.DEFAULT_SIDE_LENGTH * 1.5);
  public final Configuration configuration;
  private static final int initialize = OdeHelper.initODE2(0);
  private final DWorld world;
  private final DSpace bodySpace;
  private final DSpace signalSpace;
  private final DJointGroup collisionGroup;
  private double time;
  private final double timeStep;
  public final List<EmbodiedAgent> agents;
  private final Map<DGeom, AbstractBody> agentGeometryMapper;
  public final Map<Body, EmbodiedAgent> agentMapper;
  public final List<Body> passiveBodies;
  private final Map<DRay, SignalEmitter> signalEmitters;
  private final Map<DGeom, Boolean> signalDetectors;
  private final Map<DGeom, List<DGeom>> collisionExceptions;
  public Map<UnorderedPair<Body>, List<DDoubleBallJoint>> springJoints;
  public Map<UnorderedPair<Body>, List<DFixedJoint>> fixedJoints;
  private final DGeom terrain;

  public Ode4jEngine(Configuration configuration) {
    this.configuration = configuration;
    world = OdeHelper.createWorld();
    bodySpace = OdeHelper.createHashSpace();
    signalSpace = OdeHelper.createHashSpace();
    collisionGroup = OdeHelper.createJointGroup();
    world.setGravity(
        configuration.gravity.x(), configuration.gravity.y(), configuration.gravity.z());
    world.setERP(1d - 1e-5);
    world.setCFM(1e-5);
    agents = new ArrayList<>();
    agentGeometryMapper = new HashMap<>();
    agentMapper = new HashMap<>();
    passiveBodies = new ArrayList<>();
    signalEmitters = new HashMap<>();
    signalDetectors = new HashMap<>();
    springJoints = new HashMap<>();
    fixedJoints = new HashMap<>();
    collisionExceptions = new HashMap<>();
    // TODO ADD TERRAINS
    terrain = configuration.terrainSupplier.apply(bodySpace);
    time = 0d;
    timeStep = 1d / 60d;
    timeTickEngine = 0L;
    timeTickSignals = 0L;
    timeTickOther = 0L;
  }

  public Ode4jEngine() {
    this(DEFAULT_CONFIGURATION);
  }

  public double ERP(double springConstant, double dampingConstant) {
    return timeStep * springConstant / (timeStep * springConstant + dampingConstant);
  }

  public double CFM(double springConstant, double dampingConstant) {
    return 1d / (timeStep * springConstant + dampingConstant);
  }

  public void moveAnchors(DDoubleBallJoint joint, Vector3D position1, Vector3D position2) {
    DVector3 anchor1Position = new DVector3();
    DVector3 anchor2Position = new DVector3();
    joint.getAnchor1(anchor1Position);
    joint.getAnchor2(anchor2Position);
    joint.setAnchor1(
        anchor1Position.get0() + position1.x(),
        anchor1Position.get1() + position1.y(),
        anchor1Position.get2() + position1.z());
    joint.setAnchor2(
        anchor2Position.get0() + position2.x(),
        anchor2Position.get1() + position2.y(),
        anchor2Position.get2() + position2.z());
  }

  private void bodyCollision(Object data, DGeom o1, DGeom o2) {
    if (Objects.nonNull(collisionExceptions.get(o1)) && collisionExceptions.get(o1).contains(o2)) {
      return;
    }
    DContactBuffer contacts = new DContactBuffer(1);
    DContact contact = contacts.get(0);
    contact.surface.mode = 0;
    contact.surface.mu = OdeConstants.dInfinity;
    if (0 != OdeHelper.collide(o1, o2, 1, contacts.getGeomBuffer())) {
      OdeHelper.createContactJoint(world, collisionGroup, contact)
          .attach(o1.getBody(), o2.getBody());
      if (Objects.isNull(agentGeometryMapper.get(o1))
          || Objects.isNull(agentGeometryMapper.get(o2))
          || agentMapper.get(agentGeometryMapper.get(o1))
              != agentMapper.get(agentGeometryMapper.get(o2))) {
        if (agentGeometryMapper.get(o1) instanceof SensingBody sb) {
          for (Sensor s : sb.sensors()) {
            if (s instanceof ContactSensor cs) {
              cs.detectContact();
            }
          }
        }
        if (agentGeometryMapper.get(o2) instanceof SensingBody sb) {
          for (Sensor s : sb.sensors()) {
            if (s instanceof ContactSensor cs) {
              cs.detectContact();
            }
          }
        }
      }
    }
  }

  private void signalCollision(Object data, DGeom o1, DGeom o2) {
    if (agentGeometryMapper.get(o1) instanceof SignalDetector detector
        && signalDetectors.get(o1)
        && o2 instanceof DRay ray) {
      if (signalEmitters.get(ray) == agentGeometryMapper.get(o1)) {
        return;
      }
      DContactBuffer contacts = new DContactBuffer(1);
      if (OdeHelper.collide(o1, o2, 1, contacts.getGeomBuffer()) != 0) {
        DVector3C contactPosition = contacts.get(0).geom.pos;
        detector.readSignal(
            this,
            ray,
            new Vector3D(contactPosition.get0(), contactPosition.get1(), contactPosition.get2()));
      }
    }
  }

  public Stream<SimulationObject> allObjectsStream() {
    return Stream.concat(agents.stream(), passiveBodies.stream());
  }

  public double t() {
    return time;
  }

  public long timeTickEngine;
  public long timeTickSignals;
  public long timeTickOther;

  public InstantSnapshot tick() {
    /*if (t() % 4 < 1d / 60d) {
      System.out.printf("Current time: %.4f\n", t());
      System.out.printf("Current execution time: %.4f\n", (timeTickEngine + timeTickSignals + timeTickOther) / 1000d);
      System.out.printf("World number of non-internal springs: %d\n", agents.stream().map(EmbodiedAgent::bodyParts).flatMap(List::stream)
              .map(Body::dBody).mapToLong(b -> IntStream.range(0, b.getNumJoints()).boxed().map(b::getJoint)
                      .filter(j -> j instanceof DDoubleBallJoint).count()).sum() / 2 - 216L * agents.size());
      System.out.println("Unholy groups: ");
      springJoints.keySet().stream().filter(p -> springJoints.get(p).size() > 16)
              .forEach(p -> System.out.printf("%s ", new Pair<>(agents.indexOf(agentMapper.get(p.elements().get(0))),
                      agents.indexOf(agentMapper.get(p.elements().get(1))))));
      System.out.println();
    }*/
    long startTime = System.currentTimeMillis();
    long secondTime;
    world.quickStep(timeStep);
    collisionGroup.clear();
    bodySpace.collide(0, this::bodyCollision);
    secondTime = System.currentTimeMillis();
    timeTickEngine += secondTime - startTime;
    startTime = secondTime;
    OdeHelper.spaceCollide2(bodySpace, signalSpace, 0, this::signalCollision);
    for (DGeom signal : signalSpace.getGeoms()) {
      signal.destroy();
    }
    signalEmitters.clear();
    secondTime = System.currentTimeMillis();
    timeTickSignals += secondTime - startTime;
    time += timeStep;
    List<Action> actions = new ArrayList<>();
    for (EmbodiedAgent agent : agents) {
      actions.addAll(agent.act(this));
    }
    for (Action action : actions) {
      action.execute(this);
    }
    timeTickOther += System.currentTimeMillis() - secondTime;
    // TODO SNAPSHOT
    return null;
  }

  public DWorld world() {
    return world;
  }

  public DSpace bodySpace() {
    return bodySpace;
  }

  public DSpace signalSpace() {
    return signalSpace;
  }

  public void addAgent(EmbodiedAgent agent, Vector3D position) {
    agent.assemble(this, position);
    agents.add(agent);
    for (AbstractBody aBody : agent.components()) {
      for (Body body : aBody.bodyParts()) {
        agentMapper.put(body, agent);
        agentGeometryMapper.put(body.collisionGeometry(), aBody);
        signalDetectors.put(body.collisionGeometry(), false);
      }
      if (aBody instanceof SignalDetector detector) {
        for (Body body : detector.detectorBodies()) {
          signalDetectors.put(body.collisionGeometry(), true);
        }
      }
    }
  }

  public void addPassiveBody(Body body, Vector3D position) {
    body.assemble(this, position);
    passiveBodies.add(body);
  }

  public DDoubleBallJoint addSpringJoint(
      Body body1,
      Body body2,
      double springConstant,
      double dampingConstant,
      Vector3D position1,
      Vector3D position2) {
    UnorderedPair<Body> bodyPair = new UnorderedPair<>(body1, body2);
    DDoubleBallJoint joint = OdeHelper.createDBallJoint(world);
    joint.attach(body1.dBody(), body2.dBody());
    joint.setParam(DJoint.PARAM_N.dParamERP1, ERP(springConstant, dampingConstant));
    joint.setParam(DJoint.PARAM_N.dParamCFM1, CFM(springConstant, dampingConstant));
    moveAnchors(joint, position1, position2);
    if (Objects.isNull(springJoints.get(bodyPair))) {
      springJoints.put(bodyPair, new ArrayList<>());
    }
    springJoints.get(bodyPair).add(joint);
    return joint;
  }

  public DDoubleBallJoint addSpringJoint(
      Body body1, Body body2, double springConstant, double dampingConstant) {
    return addSpringJoint(
        body1, body2, springConstant, dampingConstant, new Vector3D(), new Vector3D());
  }

  public DFixedJoint addFixedJoint(Body body1, Body body2) {
    UnorderedPair<Body> bodyPair = new UnorderedPair<>(body1, body2);
    DFixedJoint joint = OdeHelper.createFixedJoint(world);
    joint.attach(body1.dBody(), body2.dBody());
    joint.setFixed();
    joint.setParam(DJoint.PARAM_N.dParamERP1, 1d);
    joint.setParam(DJoint.PARAM_N.dParamCFM1, 0d);
    if (Objects.isNull(fixedJoints.get(bodyPair))) {
      fixedJoints.put(bodyPair, new ArrayList<>());
    }
    fixedJoints.get(bodyPair).add(joint);
    return joint;
  }

  public void removeSpringJoints(Body body1, Body body2) {
    UnorderedPair<Body> bodyPair = new UnorderedPair<>(body1, body2);
    if (Objects.nonNull(springJoints.get(bodyPair))) {
      for (DDoubleBallJoint joint : springJoints.get(bodyPair)) {
        joint.destroy();
      }
      springJoints.remove(bodyPair);
    }
  }

  public void removeFixedJoints(Body body1, Body body2) {
    UnorderedPair<Body> bodyPair = new UnorderedPair<>(body1, body2);
    if (Objects.nonNull(fixedJoints.get(bodyPair))) {
      for (DFixedJoint joint : fixedJoints.get(bodyPair)) {
        joint.destroy();
      }
      fixedJoints.remove(bodyPair);
    }
  }

  public boolean areConnected(Attachable attachable1, Attachable attachable2) {
    for (Body b1 : attachable1.bodyParts()) {
      for (Body b2 : attachable2.bodyParts()) {
        if (Objects.nonNull(springJoints.get(new UnorderedPair<>(b1, b2)))) {
          return true;
        }
      }
    }
    return false;
  }

  public void emitSignal(SignalEmitter emitter, Vector3D direction, int channel, double value) {
    DRay ray = OdeHelper.createRay(signalSpace, configuration.nfcRange);
    ray.set(
        emitter.position(t()).x(),
        emitter.position(t()).y(),
        emitter.position(t()).z(),
        direction.x(),
        direction.y(),
        direction.z());
    // categoryBits stores the signal value as a long
    ray.setCategoryBits(Double.doubleToLongBits(value));
    // collideBits last 4 bits store the signal channel with inverted bits, while all the rest are 1
    ray.setCollideBits(~Integer.toUnsignedLong(channel));
    signalEmitters.put(ray, emitter);
  }

  public void addCollisionException(DGeom geom1, DGeom geom2) {
    if (Objects.isNull(collisionExceptions.get(geom1))) {
      collisionExceptions.put(geom1, new ArrayList<>());
    }
    collisionExceptions.get(geom1).add(geom2);
    if (Objects.isNull(collisionExceptions.get(geom2))) {
      collisionExceptions.put(geom2, new ArrayList<>());
    }
    collisionExceptions.get(geom2).add(geom1);
  }

  public void removeCollisionException(DGeom geom1, DGeom geom2) {
    if (Objects.isNull(collisionExceptions.get(geom1))
        || !collisionExceptions.get(geom1).contains(geom2)) {
      return;
    }
    collisionExceptions.get(geom1).remove(geom2);
    collisionExceptions.get(geom2).remove(geom1);
  }

  public void destroy() {
    bodySpace.destroy();
    world.destroy();
    collisionGroup.destroy();
  }
}
