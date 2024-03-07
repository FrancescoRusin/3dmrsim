package agents;

import actions.Action;
import bodies.Body;
import bodies.Sphere;
import bodies.Voxel;
import drawstuff.DrawStuff;
import engine.Ode4jEngine;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalDynamicalSystem;
import org.ode4j.ode.DDoubleBallJoint;
import org.ode4j.ode.DJoint;
import sensors.Sensor;
import test.VisualTest;
import utils.UnorderedPair;

import java.util.*;

import static drawstuff.DrawStuff.*;

public class CentralizedGridRobot extends AbstractGridRobot {
    private final NumericalDynamicalSystem<?> controller;
    private final double[] previousStepSensorOutputs;

    public CentralizedGridRobot(Voxel[][][] grid, double voxelSideLength, double voxelMass, NumericalDynamicalSystem<?> controller) {
        super(grid, voxelSideLength, voxelMass);
        this.previousStepSensorOutputs =
                new double[
                        Arrays.stream(grid).flatMap(aa -> Arrays.stream(aa).flatMap(Arrays::stream))
                                .filter(v -> !Objects.isNull(v))
                                .mapToInt(v -> v.sensors().stream().mapToInt(Sensor::outputSize).sum()).sum()];
        Arrays.fill(previousStepSensorOutputs, 0d);
        controller.checkDimension(previousStepSensorOutputs.length,
                12 * (int) Arrays.stream(grid).flatMap(aa -> Arrays.stream(aa).flatMap(Arrays::stream))
                        .filter(v -> !Objects.isNull(v)).count());
        this.controller = controller;
    }

    @Override
    public List<Action> act(Ode4jEngine engine) {
        double[] controllerOutput = controller.step(engine.t(), previousStepSensorOutputs);
        EnumMap<Voxel.Edge, Double> controlMap = new EnumMap<>(Voxel.Edge.class);
        int index = -1;
        int sensorIndex = 0;
        //noinspection ForLoopReplaceableByForEach
        for (int x = 0; x < grid.length; ++x) {
            for (int y = 0; y < grid[0].length; ++y) {
                for (int z = 0; z < grid[0][0].length; ++z) {
                    if (!Objects.isNull(grid[x][y][z])) {
                        for (Voxel.Edge e : Voxel.Edge.values()) {
                            controlMap.put(e, controllerOutput[++index]);
                        }
                        grid[x][y][z].actOnInput(controlMap);
                        for (Sensor s : grid[x][y][z].sensors()) {
                            System.arraycopy(s.sense(engine), 0, previousStepSensorOutputs, sensorIndex, s.outputSize());
                            sensorIndex += s.outputSize();
                        }
                    }
                }
            }
        }
        return List.of();
    }
}