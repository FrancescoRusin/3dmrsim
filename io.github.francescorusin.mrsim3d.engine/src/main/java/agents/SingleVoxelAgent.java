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
import java.util.function.Function;
import sensors.Sensor;
import utils.UnorderedPair;

public class SingleVoxelAgent extends Voxel implements EmbodiedAgent {
  private final double[] previousStepSensorOutputs;
  private Function<Double, EnumMap<Edge, Double>> testController;
  private boolean test;

  public SingleVoxelAgent(
      double sideLength,
      double mass,
      double springConstant,
      double dampingConstant,
      double rigidMassLengthRatio,
      double minVolumeRatio,
      double maxVolumeRatio,
      EnumSet<JointOption> jointOptions,
      String sensorConfig,
      Function<Double, EnumMap<Edge, Double>> testController, boolean test) {
    super(
        sideLength,
        mass,
        springConstant,
        dampingConstant,
        rigidMassLengthRatio,
        minVolumeRatio,
        maxVolumeRatio,
        jointOptions,
        sensorConfig);
    this.previousStepSensorOutputs =
        new double[sensors.stream().mapToInt(Sensor::outputSize).sum()];
    Arrays.fill(previousStepSensorOutputs, 0d);
    this.testController = testController;
    this.test = test;
  }

  @Override
  public List<AbstractBody> getComponents() {
    return List.of(this);
  }

  @Override
  public List<Action> act(Ode4jEngine engine) {
    // TODO IMPLEMENT CONTROLLER
    actOnInput(testController.apply(engine.t()));
    if (test && engine.t() - Math.floor(engine.t()) < 1d / 60d) {
      /*for (UnorderedPair<Vertex> extremes : vertexToVertexJoints.keySet()) {
        System.out.println(String.format("Vertices: %s %s; distance: %.3f", extremes.first(), extremes.second(), vertexToVertexJoints.get(extremes).getDistance()));
      }*/
    }
    // END TODO
    int pos = 0;
    for (Sensor s : sensors) {
      System.arraycopy(s.sense(engine), 0, previousStepSensorOutputs, pos, s.outputSize());
      pos += s.outputSize();
    }
    if (engine.t() - Math.floor(engine.t()) < 1d / 60d) {
      System.out.println(Arrays.stream(previousStepSensorOutputs).boxed().toList());
    }
    return List.of();
  }
}
