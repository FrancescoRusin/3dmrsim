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
import bodies.Voxel;
import engine.Ode4jEngine;
import java.util.*;

import io.github.ericmedvet.jsdynsym.core.numerical.NumericalDynamicalSystem;
import sensors.Sensor;

public final class SingleVoxelAgent extends Voxel implements EmbodiedAgent {
  private final double[] previousStepSensorOutputs;
  private final NumericalDynamicalSystem<?> controller;

  public SingleVoxelAgent(
      double bodyCenterToBodyCenterLength,
      double rigidBodyLength,
      double mass,
      double centralMassRatio,
      double springConstant,
      double dampingConstant,
      double sideLengthStretchRatio,
      EnumSet<JointOption> jointOptions,
      String sensorConfig,
      NumericalDynamicalSystem<?> controller) {
    super(
        bodyCenterToBodyCenterLength,
        rigidBodyLength,
        mass,
        centralMassRatio,
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
  public SingleVoxelAgent(String sensorConfig, NumericalDynamicalSystem<?> controller) {
    this(DEFAULT_BODY_CENTER_TO_BODY_CENTER_LENGTH, DEFAULT_RIGID_BODY_LENGTH, DEFAULT_MASS, DEFAULT_CENTRAL_MASS_RATIO,
            DEFAULT_SPRING_CONSTANT, DEFAULT_DAMPING_CONSTANT, DEFAULT_SIDE_LENGTH_STRETCH_RATIO,
            EnumSet.of(JointOption.EDGES_PARALLEL, JointOption.EDGES_CROSSES, JointOption.EDGES_DIAGONALS,
                    JointOption.SIDES, JointOption.INTERNAL), sensorConfig, controller);
  }

  @Override
  public List<AbstractBody> components() {
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
}
