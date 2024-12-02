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

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.glfw.GLFW.glfwShowWindow;
import static org.lwjgl.opengl.GL46.*;

import geometry.Vector3D;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.util.Objects;
import org.lwjgl.BufferUtils;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.opengl.GL;
import outcome.InstantSnapshot;

public class RealtimeViewer implements Viewer {

    @Override
    public void initialize() {
        // TODO
    }

    @Override
    public void drawTriangle(Vector3D v1, Vector3D v2, Vector3D v3) {
        // TODO
    }

    @Override
    public void drawSphere(Vector3D position, double radius) {
        // TODO
    }

    @Override
    public void drawLine(Vector3D p1, Vector3D p2) {
        // TODO
    }

    @Override
    public void draw(InstantSnapshot instant) {
        // TODO
    }

    private static String[] extractShader(String filepath) throws IOException {
        final BufferedReader reader = new BufferedReader(new FileReader(filepath));
        final String[] result = new String[2];
        String line;
        reader.readLine();
        StringBuilder builder = new StringBuilder();
        while (Objects.nonNull(line = reader.readLine())) {
            if (line.equals("#shader fragment")) {
                result[0] = builder.toString();
                builder.setLength(0);
            } else {
                builder.append(line + "\n");
            }
        }
        result[1] = builder.toString();
        return result;
    }

    private static int createShader(String[] shaders) throws InstantiationException {
        final int program = glCreateProgram();
        final int vShader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vShader, shaders[0]);
        final int[] cacher = new int[1];
        glCompileShader(vShader);
        glGetShaderiv(vShader, GL_COMPILE_STATUS, cacher);
        if (cacher[0] == GL_FALSE) {
            throw new InstantiationException("Vertex shader fucked up: " + glGetShaderInfoLog(vShader));
        }
        final int fShader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fShader, shaders[1]);
        glCompileShader(fShader);
        glGetShaderiv(fShader, GL_COMPILE_STATUS, cacher);
        if (cacher[0] == GL_FALSE) {
            throw new InstantiationException("Fragment shader fucked up: " + glGetShaderInfoLog(fShader));
        }
        glAttachShader(program, vShader);
        glAttachShader(program, fShader);
        glLinkProgram(program);
        glValidateProgram(program);
        glDetachShader(program, vShader);
        glDetachShader(program, fShader);
        glDeleteShader(vShader);
        glDeleteShader(fShader);
        return program;
    }

    private static final GLFWErrorCallback errorCallback = GLFWErrorCallback.createPrint(System.err);

    public RealtimeViewer() throws Exception {
        glfwSetErrorCallback(errorCallback);
        if (!glfwInit()) {
            throw new IllegalStateException("Unable to initialize GLFW");
        }
        String title = "HAHA!";
        int m_width = 700;
        int m_height = 700;

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE); // Sets window to be visible
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE); // Sets whether the window is resizable

        long windowID =
                glfwCreateWindow(m_width, m_height, title, 0, 0); // Does the actual window creation

        // Setup a key callback. It will be called every time a key is pressed, repeated or released.
        glfwSetKeyCallback(
                windowID,
                (window, key, scancode, action, mods) -> {
                    if (key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE)
                        glfwSetWindowShouldClose(window, true); // We will detect this in the rendering loop
                });

        glfwMakeContextCurrent(
                windowID); // glfwSwapInterval needs a context on the calling thread, otherwise will cause
        // NO_CURRENT_CONTEXT error
        GL.createCapabilities(); // Will let lwjgl know we want to use this context as the context to
        // draw with

        glfwSwapInterval(1); // How many draws to swap the buffer
        glfwShowWindow(windowID); // Shows the window

        final float[] chessboard_pixels = new float[64 * 64 * 4];
        int index = -1;
        for (int i = 0; i < 64; ++i) {
            for (int j = 0; j < 64; ++j) {
                final float checker;
                if ((i / 8) % 2 != (j / 8) % 2) {
                    checker = 1f;
                } else {
                    checker = -1f;
                }
                chessboard_pixels[++index] = checker;
                chessboard_pixels[++index] = checker;
                chessboard_pixels[++index] = checker;
                chessboard_pixels[++index] = 1.0f;
            }
        }
        final int texture = glGenTextures();
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 2);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 64, 64, 0, GL_RGBA, GL_FLOAT,
                BufferUtils.createFloatBuffer(64 * 64 * 4).put(chessboard_pixels).flip());

        final int shader =
                createShader(
                        extractShader(
                                "io.github.francescorusin.mrsim3d.engine/src/main/java/viewer/shader.txt"));
        glUseProgram(shader);
        final int textureLoc = glGetUniformLocation(shader, "uTexture");
        glUniform1i(textureLoc, 0);
        checkErrors();

        int vao = glGenVertexArrays();
        glBindVertexArray(vao);

        final float[] vertexData = new float[] {
                -.5f, -.5f, 0.0f, 0.0f,
                -.5f,  .5f, 0.0f, 1.0f,
                .5f, -.5f, 1.0f, 0.0f,
                .5f,  .5f, 1.0f, 1.0f
        };
        final int vertexBuffer = glGenBuffers();
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferData(GL_ARRAY_BUFFER, vertexData, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, false, 16, BufferUtils.createFloatBuffer(0));

        while (!glfwWindowShouldClose(windowID)) {
            checkErrors();
            glClearColor(0f, 0f, 0f, 1f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

            glfwSwapBuffers(windowID);
            glfwPollEvents();
        }
        //glDeleteTextures(texture);
    }

    private static void checkErrors() {
        int glError = glGetError();
        while (glError != GL_NO_ERROR) {
            System.out.printf("OpenGL error %d\n", glError);
            glError = glGetError();
        }
    }

    public static void main(String[] args) throws Exception {
        new RealtimeViewer();
    }
}
