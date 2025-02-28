package test; /*-
 * ========================LICENSE_START=================================
 * engine
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

import agents.CentralizedGridRobot;
import agents.SingleVoxelAgent;
import bodies.Voxel;
import drawstuff.DrawStuff;
import engine.Ode4jEngine;
import geometry.Vector3D;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalStatelessSystem;
import org.ode4j.ode.OdeHelper;

import java.util.Arrays;
import java.util.EnumSet;
import java.util.Objects;

import static drawstuff.DrawStuff.dsSetViewpoint;
import static drawstuff.DrawStuff.dsSimulationLoop;
import static drawstuff.internal.LwJGL.pause;

public class Test extends DrawStuff.dsFunctions {
    protected static final float[] xyz = {0f, -6f, 4.2600f};
    protected static final float[] hpr = {90f, -10f, 0f};
    private static final String defaultSensors = "ang-vlm-vlc-scr-cnt-nfc0";
    protected Ode4jEngine engine;
    protected CentralizedGridRobot robot;

    public static void main(String[] args) {
        new Test().demo(args);
    }

    private static int[] computeIO(String sensorConfig, int nOfVoxels, int commChannels) {
        int[] IO = new int[2];
        for (String s : sensorConfig.split("-")) {
            IO[0] +=
                    switch (s) {
                        case "ang", "vlc" -> 3;
                        case "vlm", "cnt" -> 1;
                        case "scr" -> 12;
                        default -> 0;
                    };
            if (s.matches("nfc[0-9]")) {
                IO[0] += 6;
            }
        }
        IO[0] *= nOfVoxels;
        IO[1] = nOfVoxels * (12 + 6 * commChannels);
        return IO;
    }

    private static Voxel defaultVoxel() {
        return new Voxel(
                EnumSet.of(
                        Voxel.JointOption.EDGES_PARALLEL,
                        Voxel.JointOption.EDGES_CROSSES,
                        Voxel.JointOption.EDGES_DIAGONALS,
                        Voxel.JointOption.SIDES,
                        Voxel.JointOption.INTERNAL),
                defaultSensors);
    }

    private static SingleVoxelAgent defaultSingleVoxelAgent(int commChannels) {
        final int[] IO = computeIO(defaultSensors, 1, commChannels);
        return new SingleVoxelAgent(
                defaultSensors,
                NumericalStatelessSystem.from(
                        IO[0],
                        IO[1],
                        (t, inputs) -> {
                            double[] outputArray = new double[IO[1]];
                            int index = -1;
                            for (int i = 0; i < 12; ++i) {
                                outputArray[++index] = Math.sin(4 * (t) + i * Math.PI / 4);
                            }
                            for (int j = 12; j < IO[1]; ++j) {
                                outputArray[++index] = Math.sin(t + j);
                            }
                            return outputArray;
                        }),
                1);
    }

    private static SingleVoxelAgent defaultNoCommSingleVoxelAgent() {
        final String noCommSensors = defaultSensors.substring(0, defaultSensors.length() - 5);
        final int[] IO = computeIO(noCommSensors, 1, 0);
        return new SingleVoxelAgent(
                noCommSensors,
                NumericalStatelessSystem.from(
                        IO[0],
                        IO[1],
                        (t, inputs) -> {
                            double[] outputArray = new double[12];
                            int index = -1;
                            for (int i = 0; i < 12; ++i) {
                                outputArray[++index] = Math.sin(4 * (t) + i * Math.PI / 4);
                            }
                            return outputArray;
                        }),
                0);
    }

    private static CentralizedGridRobot defaultCentralizedRobot(int commChannels) {
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
        final int nOfVoxels =
                Math.toIntExact(
                        Arrays.stream(voxelGrid)
                                .flatMap(aa -> Arrays.stream(aa).flatMap(Arrays::stream))
                                .filter(Objects::nonNull)
                                .count());
        final int[] IO = computeIO(defaultSensors, nOfVoxels, commChannels);
        return new CentralizedGridRobot(
                voxelGrid,
                commChannels,
                NumericalStatelessSystem.from(
                        IO[0],
                        IO[1],
                        (t, inputs) -> {
                            double[] outputArray = new double[IO[1]];
                            int index = -1;
                            for (int v = 0; v < nOfVoxels; ++v) {
                                for (int i = 0; i < 8; ++i) {
                                    outputArray[++index] = Math.sin(4 * t) * ((i % 4) / 2 - .5) * 2;
                                }
                                for (int i = 0; i < 4; ++i) {
                                    outputArray[++index] = 0f;
                                }
                                for (int i = 0; i < 6 * commChannels; ++i) {
                                    outputArray[++index] = Math.sin(4 * (t) + i * Math.PI / 4);
                                }
                            }
                            return outputArray;
                        }));
    }

    public void singleVoxelTest(int commChannels) {
        SingleVoxelAgent agent =
                commChannels == 0 ? defaultNoCommSingleVoxelAgent() : defaultSingleVoxelAgent(commChannels);
        engine.addAgent(agent, new Vector3D(0d, 0d, 2d));
    }

    public void multiVoxelTest(int number) {
        for (int i = 0; i < number; ++i) {
            engine.addAgent(defaultSingleVoxelAgent(1), new Vector3D(-number + 2 * i + 1, 0d, 2d));
        }
        for (int i = 0; i < number; ++i) {
            engine.allObjectsStream().forEach(o -> o.rotate(engine, new Vector3D(0d, 0d, .01)));
        }
        System.out.println();
    }

    public void hundredVoxelsTest(int commChannels) {
        for (int x = -4; x < 5; ++x) {
            for (int y = -4; y < 5; ++y) {
                engine.addAgent(
                        commChannels == 0
                                ? defaultNoCommSingleVoxelAgent()
                                : defaultSingleVoxelAgent(commChannels),
                        new Vector3D(x, y, 2d));
                engine.addAgent(
                        commChannels == 0
                                ? defaultNoCommSingleVoxelAgent()
                                : defaultSingleVoxelAgent(commChannels),
                        new Vector3D(x + 1, y + 1, 4d));
            }
        }
    }

    public void nerfedHundredVoxelsTest(int halfRows, int commChannels) {
        for (int x = -halfRows; x < halfRows + 1; ++x) {
            for (int y = -halfRows; y < halfRows + 1; ++y) {
                engine.addAgent(
                        commChannels == 0
                                ? defaultNoCommSingleVoxelAgent()
                                : defaultSingleVoxelAgent(commChannels),
                        new Vector3D(x, y, 2d));
                engine.addAgent(
                        commChannels == 0
                                ? defaultNoCommSingleVoxelAgent()
                                : defaultSingleVoxelAgent(commChannels),
                        new Vector3D(x + 1, y + 1, 4d));
            }
        }
    }

    public void robotTest(int commChannels) {
        robot = defaultCentralizedRobot(commChannels);
        engine.addAgent(robot, new Vector3D(0d, 0d, 2d));
    }

    public void demo(String[] args) {
        engine = new Ode4jEngine();
        nerfedHundredVoxelsTest(1, 1);
        dsSimulationLoop(args, 1080, 720, this);
        engine.destroy();
        OdeHelper.closeODE();
    }

    @Override
    public void start() {
        pause = true;
        dsSetViewpoint(xyz, hpr);
    }

    @Override
    public void step(boolean pause) {
        if (!pause) {
            engine.tick();
        }
    }

    @Override
    public void command(char cmd) {
        if (cmd == 'p') {
            pause = !pause;
        } else {
            engine.tick();
        }
    }

    @Override
    public void stop() {
    }
}
