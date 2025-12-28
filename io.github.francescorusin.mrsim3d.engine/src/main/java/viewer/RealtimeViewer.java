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
import bodies.Voxel;
import engine.Ode4jEngine;
import geometry.Vector3D;
import io.github.ericmedvet.jsdynsym.core.composed.OutStepped;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalDynamicalSystem;
import io.github.ericmedvet.jsdynsym.core.numerical.ann.MultiLayerPerceptron;
import snapshot.InstantSnapshot;

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

public class RealtimeViewer extends OpenGLViewer {
    protected final long window;
    protected boolean pause;
    private final double MOVEMENT_TICK = 0.05;
    private final double ROTATION_TICK = 0.01;

    public RealtimeViewer(Vector3D cameraPos, Vector3D cameraDir, Vector3D cameraUp) {
        super(cameraPos, cameraDir, cameraUp);

        // Configure GLFW
        glfwDefaultWindowHints(); // optional, the current window hints are already the default
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE); // the window will stay hidden after creation
        glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE); // the window will be resizable
        // Create the window
        window = glfwCreateWindow(800, 800, "Hello World!", NULL, NULL);
        if (window == NULL)
            throw new RuntimeException("Failed to create the GLFW window");

        // Setup a key callback. It will be called every time a key is pressed, repeated or released.
        glfwSetKeyCallback(window, (window, key, scancode, action, mods) -> {
            if (key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE) {
                glfwSetWindowShouldClose(window, true);
            }
        });

        // Setup a callback for the mouse scroll.
        glfwSetScrollCallback(window, (window, x, y) -> {
            this.cameraPos = this.cameraPos.sum(new Vector3D(0, 0, 5 * MOVEMENT_TICK * Math.signum(y)));
        });

        // Make the OpenGL context current
        glfwMakeContextCurrent(window);
        // Enable v-sync
        glfwSwapInterval(1);

        // Make the window visible
        glfwShowWindow(window);

        GLCapabilities();
    }

    public RealtimeViewer() {
        this(DEFAULT_CAMERA_POS, DEFAULT_CAMERA_DIR, DEFAULT_CAMERA_UP);
    }

    private void handleEvents() {
        glfwPollEvents();
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
            this.cameraPos = this.cameraPos.sum(new Vector3D(this.cameraDir.x(), this.cameraDir.y(), 0).normalize().times(MOVEMENT_TICK));
        }
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
            this.cameraPos = this.cameraPos.sum(this.cameraDir.vectorProduct(this.cameraUp).normalize().times(-MOVEMENT_TICK));
        }
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
            this.cameraPos = this.cameraPos.sum(new Vector3D(this.cameraDir.x(), this.cameraDir.y(), 0).normalize().times(-MOVEMENT_TICK));
        }
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
            this.cameraPos = this.cameraPos.sum(this.cameraDir.vectorProduct(this.cameraUp).normalize().times(MOVEMENT_TICK));
        }
        if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
            this.cameraDir = this.cameraDir.rotate(this.cameraDir.vectorProduct(this.cameraUp).eulerAngles(ROTATION_TICK));
        }
        if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
            this.cameraDir = this.cameraDir.rotate(
                    this.cameraUp.vectorDistance(this.cameraDir.times(this.cameraUp.scalarProduct(this.cameraDir))).eulerAngles(ROTATION_TICK)
            );
        }
        if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
            this.cameraDir = this.cameraDir.rotate(this.cameraDir.vectorProduct(this.cameraUp).eulerAngles(-ROTATION_TICK));
        }
        if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
            this.cameraDir = this.cameraDir.rotate(
                    this.cameraUp.vectorDistance(this.cameraDir.times(this.cameraUp.scalarProduct(this.cameraDir))).eulerAngles(-ROTATION_TICK)
            );
        }
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
            this.pause = !this.pause;
        }
    }

    @Override
    public void accept(InstantSnapshot instantSnapshot) {
        do {
            instantSnapshot.draw(this);
        } while (!pause);
    }

    // loop for testing; it should not be here in the final product
    private void loop(int index) {
        Vector3D origin = new Vector3D(0, 0, 0);
        Vector3D ax1 = new Vector3D(1, 0, 0);
        Vector3D ax2 = new Vector3D(0, 1, 0);
        Vector3D ax3 = new Vector3D(0, 0, 1);
        glClearColor(0.9f, 0.9f, 0.9f, 0f);
        Ode4jEngine engine = new Ode4jEngine(Ode4jEngine.Mode.DISPLAY);
        CentralizedGridRobot robot = getRobot(index);
        engine.addAgent(robot, new Vector3D(0.6, 0.6, 4.75));
        while (!glfwWindowShouldClose(window)) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            setView();
            drawLine(origin, ax1, Color.RED);
            drawLine(origin, ax2, Color.BLUE);
            drawLine(origin, ax3, Color.GREEN);
            engine.tick().draw(this);
            glfwSwapBuffers(window);
            handleEvents();
        }
    }

    private final Function<double[], CentralizedGridRobot> agentBuilder = g -> {
        MultiLayerPerceptron mlp = new MultiLayerPerceptron(
                MultiLayerPerceptron.ActivationFunction.TANH,
                80,
                new int[]{80},
                336
        );
        mlp.setParams(g);
        return new CentralizedGridRobot(
                testGrid("biped"),
                NumericalDynamicalSystem.from(
                        new OutStepped<>(mlp, 0.1),
                        80,
                        336
                )
        );
    };

    private CentralizedGridRobot getRobot(int index) {
        double[] genotype = new double[0];
        try {
            final BufferedReader reader = new BufferedReader(new FileReader("C:\\Users\\Francesco\\Desktop\\Università\\Dottorato\\Ricerca\\3dsim\\base-exp.txt"));
            for (int i = 0; i < index; ++i) {
                reader.readLine();
            }
            genotype = Arrays.stream(reader.readLine().replace("[", "").replace("]", "").split(", ")).mapToDouble(Double::parseDouble).toArray();
        } catch (Exception e) {
            e.printStackTrace();
        }
        return agentBuilder.apply(genotype);
    }

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
        for (int i = 0; i < 10; ++i) {
            new RealtimeViewer().loop(i);
        }
    }
}