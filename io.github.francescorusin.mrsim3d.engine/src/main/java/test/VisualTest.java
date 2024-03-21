package test;/*-
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

import static drawstuff.DrawStuff.*;
import static drawstuff.internal.LwJGL.pause;

import agents.CentralizedGridRobot;
import agents.EmbodiedAgent;
import agents.SingleVoxelAgent;
import bodies.Cube;
import bodies.Voxel;
import drawstuff.DrawStuff;
import engine.Ode4jEngine;
import geometry.Vector3D;

import java.util.*;

import io.github.ericmedvet.jsdynsym.core.numerical.NumericalStatelessSystem;
import org.ode4j.math.DVector3;
import org.ode4j.ode.*;

public class VisualTest extends DrawStuff.dsFunctions {
    private static final float[] xyz = {0f, -6f, 4.7600f};
    private static final float[] hpr = {90f, -10f, 0f};
    private static final String defaultSensors = "ang-vlm-vlc-scr-nfs0";
    private Ode4jEngine engine;
    private CentralizedGridRobot robot;

    public static void main(String[] args) {
        new VisualTest().demo(args);
    }

    private static Voxel defaultVoxel() {
        return new Voxel(
                EnumSet.of(Voxel.JointOption.EDGES_PARALLEL, Voxel.JointOption.EDGES_CROSSES, Voxel.JointOption.EDGES_DIAGONALS,
                        Voxel.JointOption.SIDES, Voxel.JointOption.INTERNAL),
                defaultSensors);
    }

    private static SingleVoxelAgent defaultSingleVoxelAgent() {
        return new SingleVoxelAgent(defaultSensors,
                NumericalStatelessSystem.from(25, 20,
                        (t, inputs) -> {
                            double[] outputArray = new double[20];
                            int index = -1;
                            for (int i = 0; i < 12; ++i) {
                                outputArray[++index] = Math.sin(4 * (t) + i * Math.PI / 4);
                            }
                            for (int j = 12; j < 20; ++j) {
                                outputArray[++index] = Math.sin(t + j);
                            }
                            return outputArray;
                        }), 1);
    }

    private static SingleVoxelAgent defaultNoCommSingleVoxelAgent() {
        return new SingleVoxelAgent(defaultSensors.substring(0, defaultSensors.length() - 5),
                NumericalStatelessSystem.from(19, 12,
                        (t, inputs) -> {
                            double[] outputArray = new double[12];
                            int index = -1;
                            for (int i = 0; i < 12; ++i) {
                                outputArray[++index] = -1;//Math.sin(4 * (t) + i * Math.PI / 4);
                            }
                            return outputArray;
                        }), 0);
    }

    public void singleVoxelTest() {
        engine.addAgent(defaultNoCommSingleVoxelAgent(), new Vector3D(0d, 0d, 2d));
    }

    public void multiVoxelTest(int number) {
        for (int i = 0; i < number; ++i) {
            engine.addAgent(defaultSingleVoxelAgent(), new Vector3D(-number + 2 * i, 0d, 2d));
        }
        System.out.print("Voxels: ");
        for (EmbodiedAgent agent : engine.agents()) {
            System.out.printf("%s ", agent);
        }
        System.out.println();
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
        robot = new CentralizedGridRobot(voxelGrid, 0,
                NumericalStatelessSystem.from(700, 336,
                        (t, inputs) -> {
                            double[] outputArray = new double[336];
                            int index = -1;
                            for (int v = 0; v < 28; ++v) {
                                for (int i = 0; i < 12; ++i) {
                                    outputArray[++index] = Math.sin(4 * (t) + i * Math.PI / 4);
                                }
                            }
                            return outputArray;
                        }));
        engine.addAgent(robot, new Vector3D(0d, 0d, 2d));
    }

    public void demo(String[] args) {
        engine = new Ode4jEngine();
        robotTest();
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
        engine.agents().forEach(agent -> agent.draw(this));
        engine.passiveBodies().forEach(body -> body.draw(this));
        dsSetColor(1, 1, 1);
        DVector3 placeholder1 = new DVector3();
        DVector3 placeholder2 = new DVector3();
        for (DJoint joint : engine.fixedJoints.values().stream().filter(o -> !Objects.isNull(o)).flatMap(List::stream).toList()) {
            if (joint instanceof DDoubleBallJoint doubleBallJoint) {
                doubleBallJoint.getAnchor1(placeholder1);
                doubleBallJoint.getAnchor2(placeholder2);
                dsDrawLine(placeholder1, placeholder2);
            }
            if (joint instanceof DFixedJoint fixedJoint) {
                dsDrawLine(fixedJoint.getBody(0).getPosition(), fixedJoint.getBody(1).getPosition());
            }
        }
        dsSetColor(0, 1, 0);
        for (DGeom geom : engine.signalSpace().getGeoms()) {
            if (geom instanceof DRay ray) {
                ray.get(placeholder1, placeholder2);
                placeholder2.scale(ray.getLength());
                placeholder2.add(placeholder1);
                dsDrawLine(placeholder1, placeholder2);
            }
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