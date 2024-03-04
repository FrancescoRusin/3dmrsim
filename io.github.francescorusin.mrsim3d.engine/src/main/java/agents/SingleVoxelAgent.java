/*-
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
package agents;

import actions.Action;
import bodies.AbstractBody;
import bodies.Body;
import bodies.Sphere;
import bodies.Voxel;
import drawstuff.DrawStuff;
import engine.Ode4jEngine;
import java.util.*;
import java.util.function.Function;

import io.github.ericmedvet.jsdynsym.core.numerical.NumericalDynamicalSystem;
import org.ode4j.ode.DDoubleBallJoint;
import org.ode4j.ode.DJoint;
import sensors.Sensor;
import test.VisualTest;

import static drawstuff.DrawStuff.*;
import static drawstuff.DrawStuff.dsDrawLine;

public final class SingleVoxelAgent extends Voxel implements EmbodiedAgent {
  private final double[] previousStepSensorOutputs;
  private final NumericalDynamicalSystem<?> controller;

  public SingleVoxelAgent(
      double sphereCToSphereCSideLength,
      double rigidSphereRadius,
      double mass,
      double springConstant,
      double dampingConstant,
      double sideLengthStretchRatio,
      EnumSet<JointOption> jointOptions,
      String sensorConfig,
      NumericalDynamicalSystem<?> controller) {
    super(
        sphereCToSphereCSideLength,
        rigidSphereRadius,
        mass,
        springConstant,
        dampingConstant,
        sideLengthStretchRatio,
        jointOptions,
        sensorConfig);
    this.previousStepSensorOutputs =
        new double[sensors.stream().mapToInt(Sensor::outputSize).sum()];
    Arrays.fill(previousStepSensorOutputs, 0d);
    controller.checkDimension(previousStepSensorOutputs.length, 12);
    this.controller = controller;
  }

  @Override
  public List<AbstractBody> getComponents() {
    return List.of(this);
  }

  @Override
  public List<Action> act(Ode4jEngine engine) {
    double[] controllerOutput = controller.step(engine.t(), previousStepSensorOutputs);
    EnumMap<Edge, Double> edgeOutputs = new EnumMap<>(Edge.class);
    int i = -1;
    for (Edge e : Edge.values()) {
      edgeOutputs.put(e, controllerOutput[++i]);
    }
    actOnInput(edgeOutputs);
    int pos = 0;
    for (Sensor s : sensors) {
      System.arraycopy(s.sense(engine), 0, previousStepSensorOutputs, pos, s.outputSize());
      pos += s.outputSize();
    }
    return List.of();
  }

  @Override
  public void draw(VisualTest test) {
    dsSetColor(1, 1, 0);
    dsSetTexture(DrawStuff.DS_TEXTURE_NUMBER.DS_WOOD);
    for (Body body : bodyParts()) {
      dsDrawSphere(
              body.getBody().getPosition(),
              body.getBody().getRotation(),
              ((Sphere) body).getRadius());
    }
  }
}
