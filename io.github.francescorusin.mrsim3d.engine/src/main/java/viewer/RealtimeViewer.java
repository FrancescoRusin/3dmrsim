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

import geometry.Vector3D;
import org.joml.Matrix4f;
import org.lwjgl.BufferUtils;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.opengl.GL;

import java.awt.*;
import java.nio.FloatBuffer;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.glfw.GLFW.glfwShowWindow;
import static org.lwjgl.system.MemoryUtil.NULL;

public class RealtimeViewer extends Viewer {
    private static final Vector3D DEFAULT_CAMERA_POS = new Vector3D(1, -1, 1);
    private static final Vector3D DEFAULT_CAMERA_DIR = new Vector3D(-1, 1, -1);
    private static final Vector3D DEFAULT_CAMERA_UP = new Vector3D(0, 0, 1);
    private Vector3D cameraPos;
    private Vector3D cameraDir;
    private Vector3D cameraUp;
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
                    this.cameraPos = this.cameraPos.sum(new Vector3D(-0.1, 0, 0));
                    break;
                case GLFW_KEY_S:
                    this.cameraPos = this.cameraPos.sum(new Vector3D(0, 0, -0.1));
                    break;
                case GLFW_KEY_D:
                    this.cameraPos = this.cameraPos.sum(new Vector3D(0.1, 0, 0));
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

        // Make the OpenGL context current
        glfwMakeContextCurrent(window);
        // Enable v-sync
        glfwSwapInterval(1);

        // Make the window visible
        glfwShowWindow(window);
        GL.createCapabilities();
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

    // loop for testing; it should not be here in the final product
    private void loop() {
        Vector3D v1 = new Vector3D(-0.5, -0.5, 0);
        Vector3D v2 = new Vector3D(0.5, -0.5, 0);
        Vector3D v3 = new Vector3D(0.5, 0.5, 0);
        Vector3D origin = new Vector3D(0, 0, 0);
        Vector3D ax1 = new Vector3D(1, 0, 0);
        Vector3D ax2 = new Vector3D(0, 1, 0);
        Vector3D ax3 = new Vector3D(0, 0, 1);
        Vector3D rotationT = new Vector3D(0, 0, 0.016);
        final FloatBuffer buffer = BufferUtils.createFloatBuffer(16);
        glClearColor(0.9f, 0.9f, 0.9f, 1f);
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
            v1 = v1.rotate(rotationT);
            v2 = v2.rotate(rotationT);
            v3 = v3.rotate(rotationT);
            drawTriangle(v1, v2, v3, Color.BLACK);
            glfwSwapBuffers(window);
            glfwPollEvents();
        }
    }

    @Override
    public void drawTriangle(Vector3D v1, Vector3D v2, Vector3D v3, Color color) {
        glColor3f(color.getRed(), color.getGreen(), color.getBlue());
        glBegin(GL_TRIANGLES);
        glVertex3d(v1.x(), v1.y(), v1.z());
        glVertex3d(v2.x(), v2.y(), v2.z());
        glVertex3d(v3.x(), v3.y(), v3.z());
        glEnd();
    }

    @Override
    public void drawSphere(Vector3D position, double radius, Color color) {
        // TODO
    }

    @Override
    public void drawLine(Vector3D p1, Vector3D p2, Color color) {
        glColor3f(color.getRed(), color.getGreen(), color.getBlue());
        glLineWidth(4f);
        glBegin(GL_LINES);
        glVertex3d(p1.x(), p1.y(), p1.z());
        glVertex3d(p2.x(), p2.y(), p2.z());
        glEnd();
    }

    public static void main(String[] args) throws Exception {
        new RealtimeViewer(Mode.DISPLAY).loop();
    }
}