package agents;

import actions.Action;
import bodies.AbstractBody;
import bodies.Voxel;
import engine.Ode4jEngine;
import sensors.Sensor;

import java.util.Arrays;
import java.util.EnumSet;
import java.util.List;

public class SingleVoxelAgent extends Voxel implements EmbodiedAgent {
    private final double[] previousStepSensorOutputs;

    public SingleVoxelAgent(double sideLength,
                            double mass,
                            double springConstant,
                            double dampingConstant,
                            double rigidMassLengthRatio,
                            double minVolumeRatio,
                            double maxVolumeRatio,
                            EnumSet<JointOption> jointOptions, String sensorConfig) {
        super(sideLength, mass, springConstant, dampingConstant, rigidMassLengthRatio, minVolumeRatio, maxVolumeRatio, jointOptions, sensorConfig);
        this.previousStepSensorOutputs =
                new double[sensors.stream().mapToInt(Sensor::outputSize).sum()];
        Arrays.fill(previousStepSensorOutputs, 0d);
    }

    @Override
    public List<AbstractBody> getComponents() {
        return List.of(this);
    }

    @Override
    public List<Action> act(Ode4jEngine engine) {
        // TODO IMPLEMENT CONTROLLER
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
