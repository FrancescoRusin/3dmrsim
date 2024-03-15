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

import agents.CentralizedGridRobot;
import agents.SingleVoxelAgent;
import bodies.Body;
import bodies.Cube;
import bodies.Sphere;
import bodies.Voxel;
import drawstuff.DrawStuff;
import engine.Ode4jEngine;
import geometry.Vector3D;

import java.util.*;

import io.github.ericmedvet.jsdynsym.core.numerical.NumericalStatelessSystem;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.*;

public class VisualTest extends DrawStuff.dsFunctions {
    private static final float[] xyz = {0f, -6f, 4.7600f};
    private static final float[] hpr = {90f, -10f, 0f};
    private static final String defaultSensors = "ang-vlm-vlc-scr";
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
                NumericalStatelessSystem.from(19, 12,
                        (t, inputs) -> {
                            double[] outputArray = new double[12];
                            int index = -1;
                            for (int i = 0; i < 12; ++i) {
                                outputArray[++index] = Math.sin(4 * (t) + i * Math.PI / 4);
                            }
                            return outputArray;
                        }));
    }
    public void singleVoxelTest() {
        engine.addAgent(defaultSingleVoxelAgent(), new Vector3D(0d, 0d, 2d));
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
        robot = new CentralizedGridRobot(voxelGrid, 1d, 1d,
                NumericalStatelessSystem.from(532, 336,
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
        DVector3 anchor1 = new DVector3();
        DVector3 anchor2 = new DVector3();
        for (DJoint joint : engine.fixedJoints.values().stream().filter(o -> !Objects.isNull(o)).flatMap(List::stream).toList()) {
            if (joint instanceof DDoubleBallJoint doubleBallJoint) {
                doubleBallJoint.getAnchor1(anchor1);
                doubleBallJoint.getAnchor2(anchor2);
                dsDrawLine(anchor1, anchor2);
            }
            if (joint instanceof DFixedJoint fixedJoint) {
                dsDrawLine(fixedJoint.getBody(0).getPosition(), fixedJoint.getBody(1).getPosition());
            }
        }
    }

    @Override
    public void command(char cmd) {
        engine.tick();
    }

    @Override
    public void stop() {
    }
}