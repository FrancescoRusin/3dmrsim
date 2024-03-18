package test;

import agents.CentralizedGridRobot;
import agents.SingleVoxelAgent;
import bodies.Voxel;
import engine.Ode4jEngine;
import geometry.Vector3D;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalStatelessSystem;
import test.VisualTest;

import java.util.Arrays;
import java.util.EnumSet;

public class PerformanceTest {
    private Ode4jEngine engine;

    private static Voxel defaultVoxel() {
        return new Voxel(
                EnumSet.of(Voxel.JointOption.EDGES_PARALLEL, Voxel.JointOption.EDGES_CROSSES, Voxel.JointOption.EDGES_DIAGONALS,
                        Voxel.JointOption.SIDES, Voxel.JointOption.INTERNAL),
                "ang-vlm-vlc");
    }

    private static SingleVoxelAgent defaultSingleVoxelAgent() {
        return new SingleVoxelAgent("",
                NumericalStatelessSystem.from(0, 12,
                        (t, inputs) -> {
                            double[] outputArray = new double[12];
                            int index = -1;
                            for (int i = 0; i < 4; ++i) {
                                outputArray[++index] = 0d;
                            }
                            for (int i = 0; i < 4; ++i) {
                                outputArray[++index] = Math.sin(4 * (t) + i * Math.PI / 4);
                            }
                            for (int i = 0; i < 4; ++i) {
                                outputArray[++index] = 0d;
                            }
                            return outputArray;
                        }));
    }

    public void singleVoxelTest() {
        engine.addAgent(defaultSingleVoxelAgent(), new Vector3D(0d, 0d, 1d));
    }

    public void hundredVoxelsTest() {
        for (int x = -3; x < 4; ++x) {
            for (int y = -3; y < 4; ++y) {
                engine.addAgent(defaultSingleVoxelAgent(), new Vector3D(x, y, 2d));
                engine.addAgent(defaultSingleVoxelAgent(), new Vector3D(x + 1, y + 1, 4d));
            }
        }
    }

    public void robotTest() {
        Voxel[][][] voxelGrid = new Voxel[4][3][3];
        for (int y = 0; y < 3; ++y) {
            for (int x = 0; x < 4; ++x) {
                for (int z = 1; z < 3; ++z) {
                    voxelGrid[x][y][z] = defaultVoxel();
                }
            }
        }
        voxelGrid[0][0][0] = defaultVoxel();
        voxelGrid[0][2][0] = defaultVoxel();
        voxelGrid[3][0][0] = defaultVoxel();
        voxelGrid[3][2][0] = defaultVoxel();
        engine.addAgent(new CentralizedGridRobot(voxelGrid, 1d, 1d,
                NumericalStatelessSystem.from(196, 336,
                        (t, inputs) -> {
                            double[] outputArray = new double[336];
                            int index = -1;
                            for (int v = 0; v < 28; ++v) {
                                for (int i = 0; i < 4; ++i) {
                                    outputArray[++index] = 0d;
                                }
                                for (int i = 0; i < 4; ++i) {
                                    outputArray[++index] = 0d;
                                }
                                for (int i = 0; i < 4; ++i) {
                                    outputArray[++index] = Math.sin(4 * (t) + i * Math.PI / 4);
                                }
                            }
                            return outputArray;
                        })), new Vector3D(0d, 0d, 2d));
    }

    public static void main(String[] args) {
        new PerformanceTest().unstaticMain();
    }

    public void unstaticMain() {
        long startTimeMillis;
        for (int i = 0; i < 10; ++i) {
            engine = new Ode4jEngine();
            startTimeMillis = System.currentTimeMillis();
            robotTest();
            while (engine.t() < 100) {
                engine.tick();
            }
            System.out.printf("Robot test result: %.4f seconds\n", (System.currentTimeMillis() - startTimeMillis) / 1000d);
        }
    }
}