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
import agents.EmbodiedAgent;
import bodies.Body;
import geometry.Vector3D;
import java.util.*;

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;
import utils.UnorderedPair;

public class Ode4jEngine {
  private static final int initialize = OdeHelper.initODE2(0);
  private final DWorld world;
  private final DSpace space;
  private final DJointGroup collisionGroup;
  private double time;
  private final double timeStep;
  protected List<EmbodiedAgent> agents;
  protected List<Body> passiveBodies;
  public Map<UnorderedPair<Body>, List<DDoubleBallJoint>> softJoints;
  public Map<UnorderedPair<Body>, List<DDoubleBallJoint>> rigidJoints;
  protected DGeom terrain;
  protected Vector3D DEFAULT_GRAVITY = new Vector3D(0d, 0d, -9.81);

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
            anchor1Position.get2() + position1.z()
    );
    joint.setAnchor2(
            anchor2Position.get0() + position2.x(),
            anchor2Position.get1() + position2.y(),
            anchor2Position.get2() + position2.z()
    );
  }

  private void collision(DGeom o1, DGeom o2) {
    // TODO IMPROVE COLLISION STUFF
    DContactBuffer contacts = new DContactBuffer(1);
    DContact contact = contacts.get(0);
    contact.surface.mode = 0;
    contact.surface.mu = OdeConstants.dInfinity;
    if (0 != OdeHelper.collide(o1, o2, 1, contacts.getGeomBuffer())) {
      OdeHelper.createContactJoint(world, collisionGroup, contact)
          .attach(o1.getBody(), o2.getBody());
    }
  }

  public Ode4jEngine() {
    world = OdeHelper.createWorld();
    world.setCFM(0d);
    world.setERP(0d);
    space = OdeHelper.createHashSpace(null);
    collisionGroup = OdeHelper.createJointGroup();
    world.setGravity(DEFAULT_GRAVITY.x(), DEFAULT_GRAVITY.y(), DEFAULT_GRAVITY.z());
    world.setERP(0.8);
    world.setCFM(0.05);
    agents = new ArrayList<>();
    passiveBodies = new ArrayList<>();
    softJoints = new HashMap<>();
    rigidJoints = new HashMap<>();
    // TODO ADD TERRAINS
    terrain = OdeHelper.createPlane(space, 0, 0, 1, 0);
    time = 0d;
    timeStep = 1d / 60d;
  }

  public List<EmbodiedAgent> getAgents() {
    return agents;
  }

  public List<Body> getPassiveBodies() {
    return passiveBodies;
  }

  public double t() {
    return time;
  }

  public Snapshot tick() {
    world.quickStep(timeStep);
    collisionGroup.clear();
    space.collide(space, (data, o1, o2) -> collision(o1, o2));
    time += timeStep;
    List<Action> actions = new ArrayList<>();
    for (EmbodiedAgent agent : agents) {
      actions.addAll(agent.act(this));
    }
    for (Action action : actions) {
      action.execute(this);
    }
    // TODO SNAPSHOT
    return null;
  }

  public DWorld getWorld() {
    return world;
  }

  public DSpace getSpace() {
    return space;
  }

  public void addAgent(EmbodiedAgent agent, Vector3D position) {
    agent.assemble(this, position);
    agents.add(agent);
  }

  public void addPassiveBody(Body body, Vector3D position) {
    body.assemble(this, position);
    passiveBodies.add(body);
  }

  public DDoubleBallJoint addSpringJoint(
      Body body1, Body body2, double springConstant, double dampingConstant, Vector3D position1, Vector3D position2) {
    UnorderedPair<Body> bodyPair = new UnorderedPair<>(body1, body2);
    DDoubleBallJoint joint = OdeHelper.createDBallJoint(world);
    joint.attach(body1.getBody(), body2.getBody());
    joint.setParam(DJoint.PARAM_N.dParamERP1, ERP(springConstant, dampingConstant));
    joint.setParam(DJoint.PARAM_N.dParamCFM1, CFM(springConstant, dampingConstant));
    moveAnchors(joint, position1, position2);
    if (Objects.isNull(softJoints.get(bodyPair))) {
      softJoints.put(bodyPair, new ArrayList<>());
    }
    softJoints.get(bodyPair).add(joint);
    return joint;
  }
  public DDoubleBallJoint addSpringJoint(
          Body body1, Body body2, double springConstant, double dampingConstant) {
    return addSpringJoint(body1, body2, springConstant, dampingConstant, new Vector3D(), new Vector3D());
  }
  public DDoubleBallJoint addRigidJoint(Body body1, Body body2, Vector3D position1, Vector3D position2) {
    UnorderedPair<Body> bodyPair = new UnorderedPair<>(body1, body2);
    DDoubleBallJoint joint = OdeHelper.createDBallJoint(world);
    joint.attach(body1.getBody(), body2.getBody());
    joint.setParam(DJoint.PARAM_N.dParamERP1, 1d);
    joint.setParam(DJoint.PARAM_N.dParamCFM1, 0d);
    moveAnchors(joint, position1, position2);
    if (Objects.isNull(rigidJoints.get(bodyPair))) {
      rigidJoints.put(bodyPair, new ArrayList<>());
    }
    rigidJoints.get(bodyPair).add(joint);
    return joint;
  }
  public DDoubleBallJoint addRigidJoint(Body body1, Body body2) {
    return addRigidJoint(body1, body2, new Vector3D(), new Vector3D());
  }

  public void unleash(Body body1, Body body2) {
    //TODO
  }
}
