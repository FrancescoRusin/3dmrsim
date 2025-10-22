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
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalStatelessSystem;
import org.joml.Matrix4f;
import org.lwjgl.BufferUtils;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.opengl.GL;
import org.lwjgl.stb.STBImage;
import org.lwjgl.system.MemoryStack;
import terrains.FlatTerrain;

import java.awt.*;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.Random;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.glfw.GLFW.glfwShowWindow;
import static org.lwjgl.system.MemoryStack.stackPush;
import static org.lwjgl.system.MemoryUtil.NULL;

public class RealtimeViewer extends Viewer {
    private static final Vector3D DEFAULT_CAMERA_POS = new Vector3D(5, -5, 5);
    private static final Vector3D DEFAULT_CAMERA_DIR = new Vector3D(-5, 5, -3);
    private static final Vector3D DEFAULT_CAMERA_UP = new Vector3D(0, 0, 1);
    private Vector3D cameraPos;
    private Vector3D cameraDir;
    private final Vector3D cameraUp;
    private final long window;

    public RealtimeViewer(Mode mode, Vector3D cameraPos, Vector3D cameraDir, Vector3D cameraUp) {
        super(mode);
        this.cameraPos = cameraPos;
        this.cameraDir = cameraDir.normalize();
        this.cameraUp = cameraUp;
        // Setup an error callback. The default implementation
        // will print the error message in System.err.
        GLFWErrorCallback.createPrint(System.err).set();

        // Initialize GLFW. Most GLFW functions will not work before doing this.
        if (!glfwInit())
            throw new IllegalStateException("Unable to initialize GLFW");

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
            switch (key) {
                case GLFW_KEY_ESCAPE:
                    if (action == GLFW_RELEASE) {
                        glfwSetWindowShouldClose(window, true);
                    }
                    break;
                case GLFW_KEY_W:
                    this.cameraPos = this.cameraPos.sum(new Vector3D(0, 0, 0.1));
                    break;
                case GLFW_KEY_A:
                    this.cameraPos = this.cameraPos.sum(this.cameraDir.vectorProduct(this.cameraUp).normalize().times(-0.1));
                    break;
                case GLFW_KEY_S:
                    this.cameraPos = this.cameraPos.sum(new Vector3D(0, 0, -0.1));
                    break;
                case GLFW_KEY_D:
                    this.cameraPos = this.cameraPos.sum(this.cameraDir.vectorProduct(this.cameraUp).normalize().times(0.1));
                    break;
                case GLFW_KEY_UP:
                    this.cameraDir = this.cameraDir.rotate(this.cameraDir.vectorProduct(this.cameraUp).eulerAngles(0.1));
                    break;
                case GLFW_KEY_LEFT:
                    this.cameraDir = this.cameraDir.rotate(
                            this.cameraUp.vectorDistance(this.cameraDir.times(this.cameraUp.scalarProduct(this.cameraDir))).eulerAngles(0.1)
                    );
                    break;
                case GLFW_KEY_DOWN:
                    this.cameraDir = this.cameraDir.rotate(this.cameraDir.vectorProduct(this.cameraUp).eulerAngles(-0.1));
                    break;
                case GLFW_KEY_RIGHT:
                    this.cameraDir = this.cameraDir.rotate(
                            this.cameraUp.vectorDistance(this.cameraDir.times(this.cameraUp.scalarProduct(this.cameraDir))).eulerAngles(-0.1)
                    );
                    break;
            }
        });

        glfwSetScrollCallback(window, (window, x, y) -> {
            this.cameraPos = this.cameraPos.sum(new Vector3D(this.cameraDir.x(), this.cameraDir.y(), 0).normalize().times(0.1 * Math.signum(y)));
        });

        // Make the OpenGL context current
        glfwMakeContextCurrent(window);
        // Enable v-sync
        glfwSwapInterval(1);

        // Make the window visible
        glfwShowWindow(window);
        GL.createCapabilities();

        // Make sure objects hide each other properly
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_TEXTURE_2D);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    }

    public RealtimeViewer(Mode mode) {
        this(mode, DEFAULT_CAMERA_POS, DEFAULT_CAMERA_DIR, DEFAULT_CAMERA_UP);
    }

    private Matrix4f viewMatrix() {
        Vector3D cameraTarget = cameraPos.sum(cameraDir);
        return new Matrix4f().lookAt(
                (float) cameraPos.x(), (float) cameraPos.y(), (float) cameraPos.z(),
                (float) cameraTarget.x(), (float) cameraTarget.y(), (float) cameraTarget.z(),
                (float) cameraUp.x(), (float) cameraUp.y(), (float) cameraUp.z()
        );
    }

    private Voxel[][][] testGrid(String shape) {
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
                grid[0][0][0] = new Voxel(EnumSet.allOf(Voxel.JointOption.class), "");
                grid[3][0][0] = new Voxel(EnumSet.allOf(Voxel.JointOption.class), "");
                grid[0][2][0] = new Voxel(EnumSet.allOf(Voxel.JointOption.class), "");
                grid[3][2][0] = new Voxel(EnumSet.allOf(Voxel.JointOption.class), "");
                yield grid;
            }
            default -> null;
        };
    }

    // loop for testing; it should not be here in the final product
    private void loop() {
        Vector3D origin = new Vector3D(0, 0, 0);
        Vector3D ax1 = new Vector3D(1, 0, 0);
        Vector3D ax2 = new Vector3D(0, 1, 0);
        Vector3D ax3 = new Vector3D(0, 0, 1);
        FlatTerrain terrain = new FlatTerrain();
        final FloatBuffer buffer = BufferUtils.createFloatBuffer(16);
        glClearColor(0.9f, 0.9f, 0.9f, 0f);
        Ode4jEngine engine = new Ode4jEngine();
        Random rng = new Random();
        CentralizedGridRobot robot = new CentralizedGridRobot(testGrid("biped"), NumericalStatelessSystem.from(0, 336, (t, a) -> {
            double[] output = new double[336];
            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 4; ++j) {
                    Arrays.fill(output, 132 * i + 36 * j, 132 * i + 36 * (j + 1), Math.sin(t + Math.PI * j * 0.5));
                }
            }
            Arrays.fill(output, 264, 300, Math.sin(t));
            Arrays.fill(output, 300, 336, Math.sin(t - Math.PI * 0.5));
            return output;
                }));
        engine.addAgent(robot, new Vector3D(0.6, 0.6, 4.75));
        while (!glfwWindowShouldClose(window)) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glFrustum(-0.5, 0.5, -0.5, 0.5, 0.5, 100);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            buffer.clear();
            viewMatrix().get(buffer);
            glLoadMatrixf(buffer);
            drawLine(origin, ax1, Color.RED);
            drawLine(origin, ax2, Color.BLUE);
            drawLine(origin, ax3, Color.GREEN);
            engine.tick().draw(this);
            glfwSwapBuffers(window);
            glfwPollEvents();
        }
    }

    @Override
    public int loadTexture(String filename) {
        int texID = glGenTextures();
        glBindTexture(GL_TEXTURE_2D, texID);

        try (MemoryStack stack = stackPush()) {
            IntBuffer width = stack.mallocInt(1);
            IntBuffer height = stack.mallocInt(1);
            IntBuffer channels = stack.mallocInt(1);

            STBImage.stbi_set_flip_vertically_on_load(true);
            ByteBuffer image = STBImage.stbi_load(filename, width, height, channels, 4);
            if (image == null)
                throw new RuntimeException("Image cannot be loaded: " + STBImage.stbi_failure_reason());
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width.get(0), height.get(0), 0,
                    GL_RGBA, GL_UNSIGNED_BYTE, image);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

            STBImage.stbi_image_free(image);
        }

        return texID;
    }

    @Override
    public void drawTriangle(Vector3D v1, Vector3D v2, Vector3D v3, Color color) {
        glColor3f(color.getRed() / 255f, color.getGreen() / 255f, color.getBlue() / 255f);
        glBegin(GL_TRIANGLES);
        glVertex3d(v1.x(), v1.y(), v1.z());
        glVertex3d(v2.x(), v2.y(), v2.z());
        glVertex3d(v3.x(), v3.y(), v3.z());
        glEnd();
    }

    double t = 0;

    @Override
    public void drawTexture(Vector3D v1, Vector3D v2, Vector3D v3, Vector3D v4, int texID, int hReps, int vReps) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);
        glColor3f(1f, 1f, 1f);
        glBindTexture(GL_TEXTURE_2D, texID);
        glBegin(GL_QUADS);
        glTexCoord2f(0, 0);
        glVertex3d(v1.x(), v1.y(), v1.z());
        glTexCoord2f(hReps, 0);
        glVertex3d(v2.x(), v2.y(), v2.z());
        glTexCoord2f(hReps, vReps);
        glVertex3d(v3.x(), v3.y(), v3.z());
        glTexCoord2f(0, vReps);
        glVertex3d(v4.x(), v3.y(), v4.z());
        glEnd();
        glDisable(GL_BLEND);
    }

    @Override
    public void drawSphere(Vector3D position, double radius, Color color) {
        // TODO
    }

    @Override
    public void drawLine(Vector3D p1, Vector3D p2, Color color) {
        glColor3f(color.getRed() / 255f, color.getGreen() / 255f, color.getBlue() / 255f);
        glLineWidth(4f);
        glBegin(GL_LINES);
        glVertex3d(p1.x(), p1.y(), p1.z());
        glVertex3d(p2.x(), p2.y(), p2.z());
        glEnd();
    }

    public static void main(String[] args) {
        new RealtimeViewer(Mode.DISPLAY).loop();
    }
}