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
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.opengl.GL;
import org.lwjgl.system.MemoryStack;

import java.awt.*;
import java.nio.FloatBuffer;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.glfw.GLFW.glfwShowWindow;
import static org.lwjgl.system.MemoryUtil.NULL;

public class RealtimeViewer extends Viewer {
    private final long window;

    public RealtimeViewer(Mode mode) {
        super(mode);
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
            if (key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE)
                glfwSetWindowShouldClose(window, true); // We will detect this in the rendering loop
        });

        // Make the OpenGL context current
        glfwMakeContextCurrent(window);
        // Enable v-sync
        glfwSwapInterval(1);

        // Make the window visible
        glfwShowWindow(window);
        GL.createCapabilities();
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
        //TODO ACTUALLY SOLVE THIS ROTATION MATRIX STUFF
        float[] rotationV = new float[] {1, 0, 0, 0, 0, (float) Math.cos(0.5), -(float) Math.sin(0.5), 0, 0, (float) Math.sin(0.5), (float) Math.cos(0.5), 0, 0, 0, 0, 1};
        glClearColor(0.9f, 0.9f, 0.9f, 1f);
        while (!glfwWindowShouldClose(window)) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            try (MemoryStack stack = MemoryStack.stackPush()) {
                FloatBuffer buffer = stack.mallocFloat(16);
                buffer.put(rotationV).flip();
                glLoadMatrixf(buffer);
            }
            glMatrixMode(GL_MODELVIEW);
            drawLine(origin, ax1, Color.RED);
            drawLine(origin, ax2, Color.BLUE);
            drawLine(origin, ax3, Color.GREEN);
            v1 = v1.rotate(rotationT);
            v2 = v2.rotate(rotationT);
            v3 = v3.rotate(rotationT);
            drawTriangle(v1, v2, v3, Color.WHITE);
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
        glLineWidth(2f);
        glBegin(GL_LINES);
        glVertex3d(p1.x(), p1.y(), p1.z());
        glVertex3d(p2.x(), p2.y(), p2.z());
        glEnd();
    }

    public static void main(String[] args) throws Exception {
        new RealtimeViewer(Mode.DISPLAY).loop();
    }
}