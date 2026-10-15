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
package viewer;

import agents.CentralizedGridRobot;
import agents.SingleVoxelAgent;
import bodies.Voxel;
import geometry.Vector3D;
import io.github.ericmedvet.jsdynsym.core.composed.OutStepped;
import io.github.ericmedvet.jsdynsym.core.numerical.MultivariateRealFunction;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalDynamicalSystem;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalStatelessSystem;
import io.github.ericmedvet.jsdynsym.core.numerical.ann.MultiLayerPerceptron;
import org.joml.Matrix4f;
import org.lwjgl.BufferUtils;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.opengl.GL;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;
import utils.Pair;

import java.awt.*;
import java.io.BufferedReader;
import java.io.FileReader;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.List;
import java.util.function.Function;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.system.MemoryUtil.NULL;

public class Tester {
    private final long window;

    public Tester() {
        GLFWErrorCallback.createPrint(System.err).set();
        if (!glfwInit())
            throw new IllegalStateException("Unable to initialize GLFW");
        glfwDefaultWindowHints();
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
        window = glfwCreateWindow(800, 800, "Hello World!", NULL, NULL);
        glfwSetKeyCallback(window, (window, key, scancode, action, mods) -> {
            if (key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE) {
                glfwSetWindowShouldClose(window, true);
            }
        });
        if (window == NULL)
            throw new RuntimeException("Failed to create the GLFW window");
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);
        glfwShowWindow(window);
        GL.createCapabilities();
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_TEXTURE_2D);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        OdeHelper.initODE2(0);
    }

    private void test() {
        final int ticks = 300;
        Pair<Vector3D, Vector3D> p1 = physicsLoop(ticks);
        Pair<Vector3D, Vector3D> p2 = physicsLoop(ticks);
        System.out.printf("EQUALS: %b;\n%s\n%s\n", p1.equals(p2), p1, p2);
    }

    static final double ERP = 100d / (60d * (100d / 60d + 20d));
    static final double CFM = 1d / (100d / 60d + 20d);

    private Pair<Vector3D, Vector3D> physicsLoop(int ticks) {
        Vector3D origin = new Vector3D(0, 0, 0);
        Vector3D ax1 = new Vector3D(1, 0, 0);
        Vector3D ax2 = new Vector3D(0, 1, 0);
        Vector3D ax3 = new Vector3D(0, 0, 1);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glFrustum(-0.5, 0.5, -0.5, 0.5, 0.5, 100);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glLoadMatrixf(
                new Matrix4f().lookAt(
                        5f, -5f, 5f,
                        0f, 0f, 0f,
                        0f, 0f, 1f
                ).get(BufferUtils.createFloatBuffer(16))
        );
        

        DWorld world = OdeHelper.createWorld(0);
        DSpace collisionSpace = OdeHelper.createHashSpace();
        DJointGroup collisionGroup = OdeHelper.createJointGroup();
        world.setGravity(0, 0, -9.81);
        world.setERP(1d - 1e-5);
        world.setCFM(1e-5);
        DGeom terrain = OdeHelper.createPlane(collisionSpace, 0, 0, 1, 0);
        DBody body1 = OdeHelper.createBody(world);
        body1.setPosition(-1d, 0d, 2d);
        DMass mass1 = OdeHelper.createMass();
        mass1.setBoxTotal(1, 1, 1, 1);
        body1.setMass(mass1);
        DBox box1 = OdeHelper.createBox(collisionSpace, 1, 1, 1);
        box1.setBody(body1);
        DBody body2 = OdeHelper.createBody(world);
        body2.setPosition(1d, 0d, 2d);
        DMass mass2 = OdeHelper.createMass();
        mass2.setBoxTotal(1, 1, 1, 1);
        body2.setMass(mass2);
        DBox box2 = OdeHelper.createBox(collisionSpace, 1, 1, 1);
        box2.setBody(body2);
        DDoubleBallJoint joint = OdeHelper.createDBallJoint(world);
        joint.attach(body1, body2);
        joint.setParam(DJoint.PARAM_N.dParamERP1, ERP);
        joint.setParam(DJoint.PARAM_N.dParamCFM1, CFM);
        for (int i = 0; i < ticks && !glfwWindowShouldClose(window); ++i) {
            for (int j = 0; j < 1 && !glfwWindowShouldClose(window); ++j) {
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                glClearColor(0.9f, 0.9f, 0.9f, 0f);
                drawTriangle(new Vector3D(100d, 100d, 0d), new Vector3D(-100d, 100d, 0d), new Vector3D(-100d, -100d, 0d), Color.GRAY);
                drawTriangle(new Vector3D(100d, 100d, 0d), new Vector3D(-100d, -100d, 0d), new Vector3D(100d, -100d, 0d), Color.GRAY);
                drawLine(origin, ax1, Color.RED);
                drawLine(origin, ax2, Color.BLUE);
                drawLine(origin, ax3, Color.GREEN);
                drawCube(body1, 1, Color.RED);
                drawCube(body2, 1, Color.RED);
                drawLine(
                        new Vector3D(body1.getPosition().get0(), body1.getPosition().get1(), body1.getPosition().get2()),
                        new Vector3D(body2.getPosition().get0(), body2.getPosition().get1(), body2.getPosition().get2()),
                        Color.GREEN
                );
                glfwSwapBuffers(window);
            }
            
            
            world.quickStep(1d / 60d);
            collisionGroup.clear();
            box1.collide2(box2, null, (data, g1, g2) -> {
                DContactBuffer contacts = new DContactBuffer(1);
                DContact contact = contacts.get(0);
                contact.surface.mode = 0;
                contact.surface.mu = OdeConstants.dInfinity;
                if (0 != OdeHelper.collide(g1, g2, 1, contacts.getGeomBuffer())) {
                    OdeHelper.createContactJoint(world, collisionGroup, contact)
                            .attach(g1.getBody(), g2.getBody());
                }
            });
            terrain.collide2(box1, null, (data, g1, g2) -> {
                DContactBuffer contacts = new DContactBuffer(1);
                DContact contact = contacts.get(0);
                contact.surface.mode = 0;
                contact.surface.mu = OdeConstants.dInfinity;
                if (0 != OdeHelper.collide(g1, g2, 1, contacts.getGeomBuffer())) {
                    OdeHelper.createContactJoint(world, collisionGroup, contact)
                            .attach(g1.getBody(), g2.getBody());
                }
            });
            terrain.collide2(box2, null, (data, g1, g2) -> {
                DContactBuffer contacts = new DContactBuffer(1);
                DContact contact = contacts.get(0);
                contact.surface.mode = 0;
                contact.surface.mu = OdeConstants.dInfinity;
                if (0 != OdeHelper.collide(g1, g2, 1, contacts.getGeomBuffer())) {
                    OdeHelper.createContactJoint(world, collisionGroup, contact)
                            .attach(g1.getBody(), g2.getBody());
                }
            });
            /*collisionSpace.collide(0, (data, g1, g2) -> {
                DContactBuffer contacts = new DContactBuffer(1);
                DContact contact = contacts.get(0);
                contact.surface.mode = 0;
                contact.surface.mu = OdeConstants.dInfinity;
                if (0 != OdeHelper.collide(g1, g2, 1, contacts.getGeomBuffer())) {
                    OdeHelper.createContactJoint(world, collisionGroup, contact)
                            .attach(g1.getBody(), g2.getBody());
                }
            });*/
            joint.setDistance(1 + Math.sin(i / 60d));
        }
        DVector3C p1 = body1.getPosition();
        DVector3C p2 = body2.getPosition();
        collisionGroup.destroy();
        collisionSpace.destroy();
        world.destroy();
        return new Pair<>(new Vector3D(p1.get0(), p1.get1(), p1.get2()), new Vector3D(p2.get0(), p2.get1(), p2.get2()));
    }

    private void drawLine(Vector3D p1, Vector3D p2, Color color) {
        glColor3f(color.getRed() / 255f, color.getGreen() / 255f, color.getBlue() / 255f);
        glLineWidth(4f);
        glBegin(GL_LINES);
        glVertex3d(p1.x(), p1.y(), p1.z());
        glVertex3d(p2.x(), p2.y(), p2.z());
        glEnd();
    }

    private void drawTriangle(Vector3D v1, Vector3D v2, Vector3D v3, Color color) {
        glColor3f(color.getRed() / 255f, color.getGreen() / 255f, color.getBlue() / 255f);
        glBegin(GL_TRIANGLES);
        glVertex3d(v1.x(), v1.y(), v1.z());
        glVertex3d(v2.x(), v2.y(), v2.z());
        glVertex3d(v3.x(), v3.y(), v3.z());
        glEnd();
    }

    private Vector3D cubeRotation(DBody cube) {
        DMatrix3C rotationMatrix = cube.getRotation();
        return new Vector3D(
                        Math.atan2(rotationMatrix.get21(), rotationMatrix.get22()),
                        Math.asin(-rotationMatrix.get20()),
                        Math.atan2(rotationMatrix.get10(), rotationMatrix.get00()));
    }
    
    private void drawCube(DBody cube, double sideLength, Color color) {
        Vector3D position = new Vector3D(cube.getPosition().get0(), cube.getPosition().get1(), cube.getPosition().get2());
        Vector3D rotation = cubeRotation(cube);
        Vector3D[] vs = new Vector3D[8];
        int index = -1;
        for (Vector3D v : List.of(
                new Vector3D(-1d, -1d, -1d),
                new Vector3D(-1d, 1d, -1d),
                new Vector3D(1d, 1d, -1d),
                new Vector3D(1d, -1d, -1d),
                new Vector3D(-1d, -1d, 1d),
                new Vector3D(-1d, 1d, 1d),
                new Vector3D(1d, 1d, 1d),
                new Vector3D(1d, -1d, 1d)
        )) {
            vs[++index] = v.times(.5 * sideLength).rotate(rotation).sum(position);
        }
        drawTriangle(vs[0], vs[1], vs[2], color);
        drawTriangle(vs[0], vs[2], vs[3], color);
        drawTriangle(vs[0], vs[3], vs[7], color);
        drawTriangle(vs[0], vs[4], vs[7], color);
        drawTriangle(vs[0], vs[1], vs[5], color);
        drawTriangle(vs[0], vs[4], vs[5], color);
        drawTriangle(vs[1], vs[2], vs[5], color);
        drawTriangle(vs[2], vs[5], vs[6], color);
        drawTriangle(vs[2], vs[3], vs[7], color);
        drawTriangle(vs[2], vs[6], vs[7], color);
        drawTriangle(vs[4], vs[5], vs[6], color);
        drawTriangle(vs[4], vs[6], vs[7], color);
    }

    private SingleVoxelAgent testVoxel() {
        return new SingleVoxelAgent("", NumericalStatelessSystem.from(0, 12,
                (t, a) -> new double[]{
                        Math.sin(t), Math.sin(t), Math.sin(t), Math.sin(t),
                        Math.cos(t), Math.cos(t), Math.cos(t), Math.cos(t),
                        Math.sin(t), Math.sin(t), Math.sin(t), Math.sin(t),
        }
                ));
    }

    private CentralizedGridRobot testFullRobot() {
        try {
            final BufferedReader reader = new BufferedReader(new FileReader("C:\\Users\\Francesco\\IdeaProjects\\3dmrsim\\base-exp_1.txt"));//"C:\\Users\\Francesco\\Desktop\\Università\\Dottorato\\Ricerca\\3dsim\\Res\\full-exp-%d.txt".formatted(index)));
            MultiLayerPerceptron mlp = new MultiLayerPerceptron(
                    MultiLayerPerceptron.ActivationFunction.TANH,
                    80,
                    new int[]{80},
                    336
            );
            mlp.setParams(Arrays.stream(reader.readLine().split(";")[1]
                            .replace("[", "").replace("]", "")
                            .split(", "))
                    .mapToDouble(Double::parseDouble).toArray());
            return new CentralizedGridRobot(
                    testGrid("biped"),
                    NumericalDynamicalSystem.from(
                            new OutStepped<>(mlp, 0.1),
                            80,
                            336
                    )
            );
        } catch (Exception e) {
            e.printStackTrace();
        }
        throw new RuntimeException("PORCODDIO");
    }

    private static final Function<List<Double>, CentralizedGridRobot> cubeCAgentBuilder = g -> {
        MultiLayerPerceptron mlp = new MultiLayerPerceptron(
                MultiLayerPerceptron.ActivationFunction.TANH,
                80,
                new int[]{80},
                28
        );
        mlp.setParams(g.stream().mapToDouble(d -> d).toArray());
        return new CentralizedGridRobot(
                testGrid("biped"),
                NumericalDynamicalSystem.from(
                        new OutStepped<>(
                                MultivariateRealFunction.from(
                                        i -> {
                                            final double[] mlpOutput = mlp.apply(i);
                                            final double[] output = new double[mlpOutput.length * 12];
                                            for (int j = 0; j < mlpOutput.length; ++j) {
                                                for (int k = 0; k < 12; ++k) {
                                                    output[j * 12 + k] = mlpOutput[j];
                                                }
                                            }
                                            return output;
                                        },
                                        80, 336), 0.1),
                        80,
                        336
                )
        );
    };

    private static final Function<List<Double>, CentralizedGridRobot> fullCAgentBuilder = g -> {
        MultiLayerPerceptron mlp = new MultiLayerPerceptron(
                MultiLayerPerceptron.ActivationFunction.TANH,
                80,
                new int[]{80},
                336
        );
        mlp.setParams(g.stream().mapToDouble(d -> d).toArray());
        return new CentralizedGridRobot(
                testGrid("biped"),
                NumericalDynamicalSystem.from(
                        new OutStepped<>(mlp, 0.1),
                        80,
                        336
                )
        );
    };

    private static Voxel[][][] testGrid(String shape) {
        return switch (shape) {
            case "biped" -> {
                Voxel[][][] grid = new Voxel[4][3][3];
                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 3; j++) {
                        for (int k = 1; k < 3; k++) {
                            grid[i][j][k] = new Voxel(EnumSet.allOf(Voxel.JointOption.class), "");
                        }
                    }
                }
                grid[0][0][0] = new Voxel(EnumSet.allOf(Voxel.JointOption.class), "ang-vlm-vlc-scr-cnt");
                grid[3][0][0] = new Voxel(EnumSet.allOf(Voxel.JointOption.class), "ang-vlm-vlc-scr-cnt");
                grid[0][2][0] = new Voxel(EnumSet.allOf(Voxel.JointOption.class), "ang-vlm-vlc-scr-cnt");
                grid[3][2][0] = new Voxel(EnumSet.allOf(Voxel.JointOption.class), "ang-vlm-vlc-scr-cnt");
                yield grid;
            }
            default -> null;
        };
    }

    public static void main(String[] args) {
        new Tester().test();
    }
}