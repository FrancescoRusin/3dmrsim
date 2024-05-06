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
import actions.RequestAttachment;
import actions.RequestDetachment;
import bodies.AbstractBody;
import bodies.Voxel;
import engine.Ode4jEngine;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalDynamicalSystem;
import sensors.Sensor;
import snapshot.AgentSnapshot;
import snapshot.ObjectSnapshot;
import viewer.Viewer;

import java.util.*;

public class SingleVoxelAgent extends Voxel implements EmbodiedAgent {
  private final NumericalDynamicalSystem<?> controller;
  private final int commChannels;
  private List<Action> lastStepActions;

  public SingleVoxelAgent(
          double sideLength,
          double rigidBodyLength,
          double mass,
          int commChannels,
          double centralMassRatio,
          double springConstant,
          double dampingConstant,
          double sideLengthStretchRatio,
          EnumSet<JointOption> jointOptions,
          String sensorConfig,
          NumericalDynamicalSystem<?> controller) {
    super(
            sideLength,
            rigidBodyLength,
            mass,
            centralMassRatio,
            springConstant,
            dampingConstant,
            sideLengthStretchRatio,
            jointOptions,
            sensorConfig);
    controller.checkDimension(sensors().stream().mapToInt(Sensor::outputSize).sum(), 12 + 6 * commChannels);
    this.controller = controller;
    this.commChannels = commChannels;
    this.lastStepActions = new ArrayList<>();
  }

  public SingleVoxelAgent(String sensorConfig, NumericalDynamicalSystem<?> controller, int commChannels) {
    this(DEFAULT_SIDE_LENGTH, DEFAULT_RIGID_BODY_LENGTH, DEFAULT_MASS, commChannels, DEFAULT_CENTRAL_MASS_RATIO,
            DEFAULT_SPRING_CONSTANT, DEFAULT_DAMPING_CONSTANT, DEFAULT_SIDE_LENGTH_STRETCH_RATIO,
            EnumSet.of(JointOption.EDGES_PARALLEL, JointOption.EDGES_CROSSES, JointOption.EDGES_DIAGONALS,
                    JointOption.SIDES, JointOption.INTERNAL), sensorConfig, controller);
  }

  public SingleVoxelAgent(String sensorConfig, NumericalDynamicalSystem<?> controller) {
    this(DEFAULT_SIDE_LENGTH, DEFAULT_RIGID_BODY_LENGTH, DEFAULT_MASS, 0, DEFAULT_CENTRAL_MASS_RATIO,
            DEFAULT_SPRING_CONSTANT, DEFAULT_DAMPING_CONSTANT, DEFAULT_SIDE_LENGTH_STRETCH_RATIO,
            EnumSet.of(JointOption.EDGES_PARALLEL, JointOption.EDGES_CROSSES, JointOption.EDGES_DIAGONALS,
                    JointOption.INTERNAL), sensorConfig, controller);
  }

  @Override
  public List<AbstractBody> components() {
    return List.of(this);
  }

  @Override
  public List<Action> act(Ode4jEngine engine) {
    double[] controllerOutput = controller.step(engine.t(), getSensorReadings(engine));
    EnumMap<Edge, Double> edgeOutputs = new EnumMap<>(Edge.class);
    int index = -1;
    for (Edge e : Edge.values()) {
      edgeOutputs.put(e, controllerOutput[++index]);
    }
    actOnInput(edgeOutputs);
    ++index;
    List<Action> outputActions = new ArrayList<>();
    for (int channel = 0; channel < commChannels; ++channel) {
      outputActions.addAll(super.emitSignals(engine, channel,
              Arrays.copyOfRange(controllerOutput, index, index + 6)));
      index += 6;
    }
    //TODO REMOVE TEST
    if (Math.sin(engine.t() * Math.PI / 10) > 0) {
      outputActions.addAll(Arrays.stream(Side.values()).map(Side::vertices)
              .map(l -> new RequestAttachment(this, l.stream().map(rigidBodies::get).toList()))
              .toList());
    } else {
      outputActions.addAll(Arrays.stream(Side.values()).map(Side::vertices)
              .map(l -> new RequestDetachment(this, l.stream().map(rigidBodies::get).toList()))
              .toList());
    }
    lastStepActions = new ArrayList<>(outputActions);
    return outputActions;
  }

  public record SVASnapshot(VoxelSnapshot voxelSnapshot, List<Action> actions) implements AgentSnapshot {
    @Override
    public List<ObjectSnapshot> components() {
      return List.of(voxelSnapshot);
    }

    @Override
    public void draw(Viewer viewer) {
      //TODO IMPLEMENT
    }
  }

  @Override
  public ObjectSnapshot snapshot(Ode4jEngine engine) {
    return new SVASnapshot((VoxelSnapshot) super.snapshot(engine), lastStepActions);
  }
}