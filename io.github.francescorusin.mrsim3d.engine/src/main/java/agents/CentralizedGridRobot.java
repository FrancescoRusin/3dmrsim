package agents;

import actions.Action;
import bodies.Voxel;
import engine.Ode4jEngine;
import geometry.Vector3D;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalDynamicalSystem;
import sensors.Sensor;

import java.util.*;

public class CentralizedGridRobot extends AbstractGridRobot {
    private final int commChannels;
    private final double commLength;
    private final NumericalDynamicalSystem<?> controller;
    private final double[] previousStepSensorOutputs;

    public CentralizedGridRobot(Voxel[][][] grid, double voxelSideLength, double voxelMass, int commChannels, double commLength,
                                NumericalDynamicalSystem<?> controller) {
        super(grid, voxelSideLength, voxelMass);
        this.commChannels = commChannels;
        this.previousStepSensorOutputs =
                new double[
                        Arrays.stream(grid).flatMap(aa -> Arrays.stream(aa).flatMap(Arrays::stream))
                                .filter(v -> !Objects.isNull(v))
                                .mapToInt(v -> v.sensors().stream().mapToInt(Sensor::outputSize).sum()).sum()];
        this.commLength = commLength;
        Arrays.fill(previousStepSensorOutputs, 0d);
        controller.checkDimension(previousStepSensorOutputs.length,
                ((int) Arrays.stream(grid).flatMap(aa -> Arrays.stream(aa).flatMap(Arrays::stream))
                        .filter(v -> !Objects.isNull(v)).count() * (12 + 8 * commChannels)));
        this.controller = controller;
    }

    public CentralizedGridRobot(Voxel[][][] grid, double voxelSideLength, double voxelMass, int commChannels,
                                NumericalDynamicalSystem<?> controller) {
        this(grid, voxelSideLength, voxelMass, commChannels, DEFAULT_COMM_LENGTH, controller);
    }

    public CentralizedGridRobot(Voxel[][][] grid, double voxelSideLength, double voxelMass,
                                NumericalDynamicalSystem<?> controller) {
        this(grid, voxelSideLength, voxelMass, 0, 0, controller);
    }

    public CentralizedGridRobot(Voxel[][][] grid, int commChannels, NumericalDynamicalSystem<?> controller) {
        this(grid, Voxel.DEFAULT_BODY_CENTER_TO_BODY_CENTER_LENGTH, Voxel.DEFAULT_MASS, commChannels,
                DEFAULT_COMM_LENGTH, controller);
    }

    public CentralizedGridRobot(Voxel[][][] grid, NumericalDynamicalSystem<?> controller) {
        this(grid, Voxel.DEFAULT_BODY_CENTER_TO_BODY_CENTER_LENGTH, Voxel.DEFAULT_MASS, controller);
    }

    @Override
    public List<Action> act(Ode4jEngine engine) {
        EnumMap<Voxel.Edge, Double> controlMap = new EnumMap<>(Voxel.Edge.class);
        int sensorIndex = 0;
        for (Voxel[][] voxelMatrix : grid) {
            for (Voxel[] voxelRow : voxelMatrix) {
                for (Voxel voxel : voxelRow) {
                    if (!Objects.isNull(voxel)) {
                        for (Sensor s : voxel.sensors()) {
                            System.arraycopy(s.sense(engine), 0, previousStepSensorOutputs, sensorIndex, s.outputSize());
                            sensorIndex += s.outputSize();
                        }
                    }
                }
            }
        }
        double[] controllerOutput = controller.step(engine.t(), previousStepSensorOutputs);
        int index = -1;
        List<Action> outputActions = new ArrayList<>();
        for (Voxel[][] voxelMatrix : grid) {
            for (Voxel[] voxelRow : voxelMatrix) {
                for (Voxel voxel : voxelRow) {
                    if (!Objects.isNull(voxel)) {
                        for (Voxel.Edge e : Voxel.Edge.values()) {
                            controlMap.put(e, controllerOutput[++index]);
                        }
                        voxel.actOnInput(controlMap);
                        ++index;
                        for (int channel = 0; channel < commChannels; ++channel) {
                            outputActions.addAll(voxel.emitSignals(engine, commLength, channel,
                                    Arrays.copyOfRange(controllerOutput, index, index + 6)));
                            index += 6;
                        }
                        for (Sensor s : voxel.sensors()) {
                            System.arraycopy(s.sense(engine), 0, previousStepSensorOutputs, sensorIndex, s.outputSize());
                            sensorIndex += s.outputSize();
                        }
                    }
                }
            }
        }
        return outputActions;
    }
}