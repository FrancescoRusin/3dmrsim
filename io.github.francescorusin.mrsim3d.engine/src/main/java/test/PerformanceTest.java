package test;

import agents.SingleVoxelAgent;
import bodies.Voxel;
import engine.Ode4jEngine;
import geometry.Vector3D;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalStatelessSystem;

import java.util.EnumSet;

public class PerformanceTest {
    public static void main(String[] args) {
        collisionTest();
    }

    private static Voxel defaultVoxel() {
        return new Voxel(
                EnumSet.of(Voxel.JointOption.EDGES_PARALLEL, Voxel.JointOption.EDGES_CROSSES, Voxel.JointOption.EDGES_DIAGONALS,
                        Voxel.JointOption.SIDES, Voxel.JointOption.INTERNAL),
                "ang-vlm-vlc");
    }

    private static SingleVoxelAgent defaultSingleVoxelAgent() {
        return new SingleVoxelAgent(1.4, 0.3, 1d, .5,
                100d, 20d, 0.2,
                EnumSet.of(Voxel.JointOption.EDGES_PARALLEL, Voxel.JointOption.EDGES_CROSSES, Voxel.JointOption.EDGES_DIAGONALS), "",
                NumericalStatelessSystem.from(0, 12,
                        (t, inputs) -> {
                            double[] outputArray = new double[12];
                            int index = -1;
                            for (int i = 0; i < 4; ++i) {
                                outputArray[++index] = 0d;
                            }
                            for (int i = 0; i < 4; ++i) {
                                outputArray[++index] = 0d;
                            }
                            for (int i = 0; i < 4; ++i) {
                                outputArray[++index] = Math.sin(t);
                            }
                            return outputArray;
                        }));
    }

    private static void collisionTest() {
        System.out.println("With collision:");
        long startTime;
        Ode4jEngine engine;
        for (int i = 0; i < 3; ++i) {
            startTime = System.currentTimeMillis();
            engine = new Ode4jEngine();
            for (int x = 0; x < 10; ++x) {
                for (int y = 0; y < 10; ++y) {
                    engine.addAgent(defaultSingleVoxelAgent(), new Vector3D(x * 2d, y * 2d, 2d));
                }
            }
            while (engine.t() < 100d) {
                engine.tick();
            }
            engine.destroy();
            System.out.printf("Elapsed time: %.4f\n", (System.currentTimeMillis() - startTime) / 1000d);
        }
        System.out.println("Without collision:");
        for (int i = 0; i < 3; ++i) {
            startTime = System.currentTimeMillis();
            engine = new Ode4jEngine();
            for (int x = 0; x < 10; ++x) {
                for (int y = 0; y < 10; ++y) {
                    engine.addAgent(defaultSingleVoxelAgent(), new Vector3D(x * 2d, y * 2d, 2d));
                }
            }
            while (engine.t() < 100d) {
                engine.noCollisionTick();
            }
            System.out.printf("Elapsed time: %.4f\n", (System.currentTimeMillis() - startTime) / 1000d);
        }
    }
}