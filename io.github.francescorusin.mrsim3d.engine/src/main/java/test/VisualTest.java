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
import agents.EmbodiedAgent;
import bodies.Body;
import bodies.Cube;
import bodies.Sphere;
import bodies.Voxel;
import drawstuff.DrawStuff;
import engine.Ode4jEngine;
import geometry.Vector3D;

import java.util.EnumSet;

import io.github.ericmedvet.jsdynsym.core.numerical.NumericalStatelessSystem;
import org.ode4j.math.DVector3;
import org.ode4j.ode.*;

public class VisualTest extends DrawStuff.dsFunctions {
    private static final float[] xyz = {0f, -4f, 0.7600f};
    private static final float[] hpr = {90f, -10f, 0f};
    private final Ode4jEngine engine = new Ode4jEngine();
    private CentralizedGridRobot robot;

    public static void main(String[] args) {
        new VisualTest().demo(args);
    }

    private static Voxel defaultVoxel() {
        return new Voxel(
                EnumSet.of(Voxel.JointOption.EDGES, Voxel.JointOption.SIDES, Voxel.JointOption.INTERNAL),
                "ang-vlm-vlc");
    }

    public void demo(String[] args) {
        Voxel[][][] voxelGrid = new Voxel[3][4][3];
        for (int x = 0; x < 3; ++x) {
            for (int y = 0; y < 4; ++y) {
                for (int z = 1; z < 3; ++z) {
                    voxelGrid[x][y][z] = defaultVoxel();
                }
            }
            voxelGrid[x][0][0] = defaultVoxel();
            voxelGrid[x][3][0] = defaultVoxel();
        }
        robot = new CentralizedGridRobot(voxelGrid, 1d, 1d,
                NumericalStatelessSystem.from(210, 360,
                        (t, inputs) -> {
                            double[] outputArray = new double[360];
                            int index = -1;
                            for (int v = 0; v < 30; ++v) {
                                for (int i = 0; i < 12; ++i) {
                                    outputArray[++index] = 0d;
                                }
                                for (int i = 0; i < 4; ++i) {
                                    //outputArray[++index] = Math.sin(t);
                                }
                            }
                            return outputArray;
                        }));
        dsSimulationLoop(args, 1080, 720, this);
        engine.getSpace().destroy();
        engine.getWorld().destroy();
        OdeHelper.closeODE();
    }

    @Override
    public void start() {
        //engine.addAgent(robot, new Vector3D(0d, 0d, 2d));
        engine.addPassiveBody(new Sphere(.2d, 1d), new Vector3D(0d, 0d, 1d));
        dsSetViewpoint(xyz, hpr);
    }

    @Override
    public void step(boolean pause) {
        if (!pause) {
            engine.tick();
        }
        engine.getAgents().forEach(agent -> agent.draw(this));
        dsSetColor(1, 1, 1);
        dsSetTexture(DS_TEXTURE_NUMBER.DS_CHECKERED);
        for (Body body : engine.passiveBodies) {
            if (body instanceof Cube cube) {
                dsDrawBox(cube.getBody().getPosition(), cube.getBody().getRotation(), new DVector3(1d, 1d, 1d));
            } else if (body instanceof Sphere sphere) {
                dsDrawSphere(sphere.getBody().getPosition(), sphere.getBody().getRotation(), sphere.getRadius());
            }
            System.out.println(body.boundingBox(engine.t()));
        }
        for (DJoint joint : engine.softJoints.values()) {
            if (joint instanceof DDoubleBallJoint doubleBallJoint) {
                dsDrawLine(
                        doubleBallJoint.getBody(0).getPosition(), doubleBallJoint.getBody(1).getPosition());
            }
        }
        dsSetColor(1, 1, 1);
        for (DJoint joint : engine.rigidJoints.values()) {
            if (joint instanceof DDoubleBallJoint doubleBallJoint) {
                dsDrawLine(
                        doubleBallJoint.getBody(0).getPosition(), doubleBallJoint.getBody(1).getPosition());
            }
        }
    }

    @Override
    public void command(char cmd) {
    }

    @Override
    public void stop() {
    }
}