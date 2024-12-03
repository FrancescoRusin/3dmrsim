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
import bodies.Voxel;
import engine.Ode4jEngine;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalDynamicalSystem;
import sensors.Sensor;

import java.util.*;

public class CentralizedGridRobot extends AbstractGridRobot {
    private final int commChannels;
    private final NumericalDynamicalSystem<?> controller;
    private final double[] previousStepSensorOutputs;

    public CentralizedGridRobot(
            Voxel[][][] grid,
            double voxelSideLength,
            double voxelMass,
            int commChannels,
            NumericalDynamicalSystem<?> controller) {
        super(grid, voxelSideLength, voxelMass);
        this.commChannels = commChannels;
        this.previousStepSensorOutputs =
                new double
                        [Arrays.stream(grid)
                        .flatMap(aa -> Arrays.stream(aa).flatMap(Arrays::stream))
                        .filter(Objects::nonNull)
                        .mapToInt(v -> v.sensors().stream().mapToInt(Sensor::outputSize).sum())
                        .sum()];
        Arrays.fill(previousStepSensorOutputs, 0d);
        controller.checkDimension(
                previousStepSensorOutputs.length,
                ((int)
                        Arrays.stream(grid)
                                .flatMap(aa -> Arrays.stream(aa).flatMap(Arrays::stream))
                                .filter(Objects::nonNull)
                                .count()
                        * (12 + 6 * commChannels)));
        this.controller = controller;
    }

    public CentralizedGridRobot(
            Voxel[][][] grid,
            double voxelSideLength,
            double voxelMass,
            NumericalDynamicalSystem<?> controller) {
        this(grid, voxelSideLength, voxelMass, 0, controller);
    }

    public CentralizedGridRobot(
            Voxel[][][] grid, int commChannels, NumericalDynamicalSystem<?> controller) {
        this(grid, Voxel.DEFAULT_SIDE_LENGTH, Voxel.DEFAULT_MASS, commChannels, controller);
    }

    public CentralizedGridRobot(Voxel[][][] grid, NumericalDynamicalSystem<?> controller) {
        this(grid, Voxel.DEFAULT_SIDE_LENGTH, Voxel.DEFAULT_MASS, controller);
    }

    @Override
    public List<Action> act(Ode4jEngine engine) {
        EnumMap<Voxel.Edge, Double> controlMap = new EnumMap<>(Voxel.Edge.class);
        int sensorIndex = 0;
        for (Voxel[][] voxelMatrix : grid) {
            for (Voxel[] voxelRow : voxelMatrix) {
                for (Voxel voxel : voxelRow) {
                    if (Objects.nonNull(voxel)) {
                        for (Sensor s : voxel.sensors()) {
                            System.arraycopy(
                                    s.sense(engine), 0, previousStepSensorOutputs, sensorIndex, s.outputSize());
                            sensorIndex += s.outputSize();
                        }
                    }
                }
            }
        }
        double[] controllerOutput = controller.step(engine.t(), previousStepSensorOutputs);
        int index = 0;
        List<Action> outputActions = new ArrayList<>();
        for (Voxel[][] voxelMatrix : grid) {
            for (Voxel[] voxelRow : voxelMatrix) {
                for (Voxel voxel : voxelRow) {
                    if (Objects.nonNull(voxel)) {
                        for (Voxel.Edge e : Voxel.Edge.values()) {
                            controlMap.put(e, controllerOutput[index++]);
                        }
                        voxel.actOnInput(controlMap);
                        for (int channel = 0; channel < commChannels; ++channel) {
                            outputActions.addAll(
                                    voxel.emitSignals(
                                            engine, channel, Arrays.copyOfRange(controllerOutput, index, index + 6)));
                            index += 6;
                        }
                    }
                }
            }
        }
        return outputActions;
    }
}
