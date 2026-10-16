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
import engine.Ode4jEngine;
import geometry.Vector3D;
import io.github.ericmedvet.jsdynsym.core.composed.OutStepped;
import io.github.ericmedvet.jsdynsym.core.numerical.MultivariateRealFunction;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalDynamicalSystem;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalStatelessSystem;
import io.github.ericmedvet.jsdynsym.core.numerical.ann.MultiLayerPerceptron;
import org.lwjgl.BufferUtils;
import org.lwjgl.stb.STBImageWrite;
import org.lwjgl.system.MemoryStack;
import snapshot.InstantSnapshot;
import tasks.Locomotion;

import java.awt.*;
import java.io.BufferedReader;
import java.io.FileReader;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Function;
import java.util.stream.Stream;

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
            if (key == GLFW_KEY_P && action == GLFW_RELEASE) {
                screenshot();
            }
        });

        // Setup a callback for the mouse scroll
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

    int counter = 0;

    public void screenshot() {
        ByteBuffer pixels = BufferUtils.createByteBuffer(800 * 800 * 4);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadBuffer(GL_FRONT);
        glReadPixels(0, 0,800, 800, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
        STBImageWrite.stbi_flip_vertically_on_write(true);
        try (MemoryStack stack = MemoryStack.stackPush()) {
            boolean success = STBImageWrite.stbi_write_png("screenshot_%d.png".formatted(counter), 800, 800, 4, pixels, 3200);
            if (!success) {
                throw new RuntimeException("PNG saving error");
            }
        }
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
        CentralizedGridRobot robot = fullCRobot(index);
        engine.addAgent(robot, new Vector3D(0, 0, 0));
        robot.translate(engine, new Vector3D(0, 0, 1 - robot.boundingBox(0).min().z()));
        while (engine.t() < 30 && !glfwWindowShouldClose(window)) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            setView();
            drawLine(origin, ax1, Color.RED);
            drawLine(origin, ax2, Color.BLUE);
            drawLine(origin, ax3, Color.GREEN);
            engine.tick().draw(this);
            glfwSwapBuffers(window);
            handleEvents();
        }
        while (!glfwWindowShouldClose(window)) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            setView();
            drawLine(origin, ax1, Color.RED);
            drawLine(origin, ax2, Color.BLUE);
            drawLine(origin, ax3, Color.GREEN);
            engine.currentState().draw(this);
            glfwSwapBuffers(window);
            handleEvents();
        }
    }

    // loop for testing; it should not be here in the final product
    private void stupidloop() {
        int texID = loadTexture("C:\\Users\\Francesco\\IdeaProjects\\3dmrsim\\io.github.francescorusin.mrsim3d.engine\\src\\main\\resources\\textures\\Chessboard.jpg");
        while (!glfwWindowShouldClose(window)) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(0.9f, 0.9f, 0.9f, 0f);
            setView();
            drawTexture(
                    new Vector3D(-100, -100, 0), new Vector3D(100, -100, 0),
                    new Vector3D(100, 100, 0), new Vector3D(-100, 100, 0),
                    texID, 1, 1);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glEnable(GL_BLEND);
            for (Voxel.Tetrahedron tetrahedron : Voxel.Tetrahedron.values()) {
                for (int i = 0; i < 4; ++i) {
                    List<Vector3D> vertexPosCopy = new ArrayList<>(Stream.of(tetrahedron.v1, tetrahedron.v2, tetrahedron.v3, tetrahedron.v4)
                            .map(v -> new Vector3D(v.ordinal() % 2, Math.floor(v.ordinal() / 2d) % 2, Math.floor(v.ordinal() / 4d) + 1))
                            .toList());
                    vertexPosCopy.remove(i);
                    if (vertexPosCopy.stream().filter(v -> v.x() == vertexPosCopy.getFirst().x()).toList().size() != 3 ||
                            vertexPosCopy.stream().filter(v -> v.y() == vertexPosCopy.getFirst().y()).toList().size() != 3 ||
                            vertexPosCopy.stream().filter(v -> v.z() == vertexPosCopy.getFirst().z()).toList().size() != 3) {
                        drawTriangle(vertexPosCopy.get(0), vertexPosCopy.get(1), vertexPosCopy.get(2), Color.RED);
                    }
                    for (int j = 0; j < 3; ++j) {
                        for (int k = 0; k < j; ++k) {
                            drawLine(vertexPosCopy.get(j), vertexPosCopy.get(k), Color.DARK_GRAY);
                        }
                    }
                }
            }
            glfwSwapBuffers(window);
            handleEvents();
        }
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

    private CentralizedGridRobot cubeCRobot(int index) {
        List<Double> genotype = new ArrayList<>();
        try {
            final BufferedReader reader = new BufferedReader(new FileReader("C:\\Users\\Francesco\\Desktop\\Università\\Dottorato\\Ricerca\\3dsim\\Res\\cube-exp-%d.txt".formatted(index)));
            genotype = Arrays.stream(reader.readLine().split(";")[1]
                            .replace("[", "").replace("]", "")
                            .split(", "))
                    .mapToDouble(Double::parseDouble).boxed().toList();
        } catch (Exception e) {
            e.printStackTrace();
        }
        return cubeCAgentBuilder.apply(genotype);
    }

    private CentralizedGridRobot fullCRobot(int index) {
        List<Double> genotype = new ArrayList<>();
        try {
            final BufferedReader reader = new BufferedReader(new FileReader("C:\\Users\\Francesco\\Desktop\\Università\\Dottorato\\Ricerca\\3dsim\\Res\\full-exp-%d.txt".formatted(index)));
            genotype = Arrays.stream(reader.readLine().split(";")[1]
                            .replace("[", "").replace("]", "")
                            .split(", "))
                    .mapToDouble(Double::parseDouble).boxed().toList();
        } catch (Exception e) {
            e.printStackTrace();
        }
        return fullCAgentBuilder.apply(genotype);
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
        new RealtimeViewer().stupidloop();
    }
}