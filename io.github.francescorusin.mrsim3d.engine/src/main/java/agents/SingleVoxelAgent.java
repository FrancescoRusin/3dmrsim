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
import actions.EmitSignal;
import bodies.AbstractBody;
import bodies.Voxel;
import engine.Ode4jEngine;
import java.util.*;

import io.github.ericmedvet.jsdynsym.core.numerical.NumericalDynamicalSystem;
import sensors.Sensor;

public final class SingleVoxelAgent extends Voxel implements EmbodiedAgent {
  private final double[] previousStepSensorOutputs;
  private final NumericalDynamicalSystem<?> controller;
  private final int commChannels;
  private final double commLength;

  public SingleVoxelAgent(
          double bodyCenterToBodyCenterLength,
          double rigidBodyLength,
          double mass,
          int commChannels,
          double commLength,
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
            new double[sensors().stream().mapToInt(Sensor::outputSize).sum()];
    Arrays.fill(previousStepSensorOutputs, 0d);
    controller.checkDimension(previousStepSensorOutputs.length, 12 + 8 * commChannels);
    this.controller = controller;
    this.commChannels = commChannels;
    this.commLength = commLength;
  }

  public SingleVoxelAgent(String sensorConfig, NumericalDynamicalSystem<?> controller, int commChannels) {
    this(DEFAULT_BODY_CENTER_TO_BODY_CENTER_LENGTH, DEFAULT_RIGID_BODY_LENGTH, DEFAULT_MASS, commChannels, DEFAULT_COMM_LENGTH,
            DEFAULT_CENTRAL_MASS_RATIO,
            DEFAULT_SPRING_CONSTANT, DEFAULT_DAMPING_CONSTANT, DEFAULT_SIDE_LENGTH_STRETCH_RATIO,
            EnumSet.of(JointOption.EDGES_PARALLEL, JointOption.EDGES_CROSSES, JointOption.EDGES_DIAGONALS,
                    JointOption.INTERNAL), sensorConfig, controller);
  }

  public SingleVoxelAgent(String sensorConfig, NumericalDynamicalSystem<?> controller) {
    this(DEFAULT_BODY_CENTER_TO_BODY_CENTER_LENGTH, DEFAULT_RIGID_BODY_LENGTH, DEFAULT_MASS, 0, DEFAULT_COMM_LENGTH,
            DEFAULT_CENTRAL_MASS_RATIO, DEFAULT_SPRING_CONSTANT, DEFAULT_DAMPING_CONSTANT, DEFAULT_SIDE_LENGTH_STRETCH_RATIO,
            EnumSet.of(JointOption.EDGES_PARALLEL, JointOption.EDGES_CROSSES, JointOption.EDGES_DIAGONALS,
                    JointOption.INTERNAL), sensorConfig, controller);
  }

  @Override
  public List<AbstractBody> components() {
    return List.of(this);
  }

  @Override
  public List<Action> act(Ode4jEngine engine) {
    int pos = 0;
    for (Sensor s : sensors()) {
      System.arraycopy(s.sense(engine), 0, previousStepSensorOutputs, pos, s.outputSize());
      pos += s.outputSize();
    }
    double[] controllerOutput = controller.step(engine.t(), previousStepSensorOutputs);
    EnumMap<Edge, Double> edgeOutputs = new EnumMap<>(Edge.class);
    int index = -1;
    for (Edge e : Edge.values()) {
      edgeOutputs.put(e, controllerOutput[++index]);
    }
    actOnInput(edgeOutputs);
    ++index;
    List<Action> outputActions = new ArrayList<>();
    for (int channel = 0; channel < commChannels; ++channel) {
      outputActions.addAll(super.emitSignals(engine, commLength, channel,
              Arrays.copyOfRange(controllerOutput, index, index + 6)));
      index += 6;
    }
    return outputActions;
  }
}