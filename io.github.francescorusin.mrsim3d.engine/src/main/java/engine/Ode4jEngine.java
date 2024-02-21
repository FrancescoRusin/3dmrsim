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
import org.ode4j.ode.*;
import utils.Pair;

public class Ode4jEngine {
  private static int initialize = OdeHelper.initODE2(0);
  private final DWorld world;
  private final DSpace space;
  private double time;
  private final double timeStep;
  protected List<EmbodiedAgent> agents;
  protected List<Body> passiveBodies;

  protected Map<Pair<Body, Body>, DDoubleBallJoint> joints;
  protected DGeom terrain;
  protected Vector3D DEFAULT_GRAVITY = new Vector3D(0d, 0d, -9.81);

  private double ERP(double springConstant, double dampingConstant) {
    return timeStep * springConstant / (timeStep * springConstant + dampingConstant);
  }

  private double CFM(double springConstant, double dampingConstant) {
    return 1d / (timeStep * springConstant + dampingConstant);
  }

  public Ode4jEngine() {
    world = OdeHelper.createWorld();
    space = OdeHelper.createHashSpace(null);
    world.setGravity(DEFAULT_GRAVITY.x(), DEFAULT_GRAVITY.y(), DEFAULT_GRAVITY.z());
    agents = new ArrayList<>();
    passiveBodies = new ArrayList<>();
    joints = new HashMap<>();
    // TODO ADD TERRAINS
    terrain = OdeHelper.createPlane(space, 0, 0, 1, 0);
    time = 0d;
    timeStep = 1d / 60d;
  }

  public List<EmbodiedAgent> getAgents() {
    return agents;
  }

  private List<Body> getPassiveBodies() {
    return passiveBodies;
  }

  public double t() {
    return time;
  }

  public Snapshot tick() {
    world.quickStep(timeStep);
    time += timeStep;
    List<Action> actions = new ArrayList<>();
    for (EmbodiedAgent agent : agents) {
      actions.addAll(agent.act());
    }
    for (Action action : actions) {
      action.execute(this);
    }
    throw new IllegalArgumentException("TODO!");
  }

  public DWorld getWorld() {
    return world;
  }
  public DSpace getSpace() {return space;}

  void addAgent(EmbodiedAgent agent, Vector3D position) {
    agent.assemble(this, position);
    agents.add(agent);
  }

  public void addPassiveBody(Body body, Vector3D position) {
    body.assemble(this, position);
    passiveBodies.add(body);
  }

  public void addSpringJoint(Body body1, Body body2, double springConstant, double dampingConstant) {
    Pair<Body, Body> bodyPair = new Pair<>(body1, body2);
    if (joints.containsKey(bodyPair))
      return;
    DDoubleBallJoint joint = OdeHelper.createDBallJoint(world);
    joint.setParam(DJoint.PARAM_N.dParamERP1, ERP(springConstant, dampingConstant));
    joint.setParam(DJoint.PARAM_N.dParamCFM1, CFM(springConstant, dampingConstant));
    joint.attach(body1.getBody(), body2.getBody());
    joints.put(bodyPair, joint);
  }

  public void unleash(Body body1, Body body2) {
    Pair<Body, Body> bodyPair = new Pair<>(body1, body2);
    if (!joints.containsKey(bodyPair))
      return;
    joints.get(bodyPair).destroy();
    joints.remove(bodyPair);
  }
}
