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
import org.lwjgl.BufferUtils;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.opengl.GL;
import org.lwjgl.opengl.GLUtil;
import org.lwjgl.system.Callback;
import outcome.InstantSnapshot;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Objects;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL33.*;

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
        int m_width = 800;
        int m_height = 800;

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE); // Sets window to be visible
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE); // Sets whether the window is resizable
        glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);

        long windowID =
                glfwCreateWindow(m_width, m_height, title, 0, 0); // Does the actual window creation

        // Setup a key callback. It will be called every time a key is pressed, repeated or released.
        glfwSetKeyCallback(
                windowID,
                (window, key, scancode, action, mods) -> {
                    if (key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE)
                        glfwSetWindowShouldClose(window, true); // We will detect this in the rendering loop
                });

        glfwMakeContextCurrent(windowID);
        glfwSwapInterval(1);
        glfwShowWindow(windowID);
        GL.createCapabilities();
        Callback debugProc = GLUtil.setupDebugMessageCallback();

        final int width = 256;
        final int height = 256;
        final int stepW = width / 8;
        final int stepH = height / 8;
        final byte[] chessboard_pixels = new byte[width * height * 4];
        int index = -1;
        for (int i = 0; i < width; ++i) {
            for (int j = 0; j < height; ++j) {
                final byte checker;
                if ((i / stepW) % 2 != (j / stepH) % 2) {
                    checker = (byte) 255;
                } else {
                    checker = (byte) 127;
                }
                chessboard_pixels[++index] = checker;
                chessboard_pixels[++index] = checker;
                chessboard_pixels[++index] = checker;
                chessboard_pixels[++index] = (byte) 255;
            }
        }

        /*BufferedImage ruby = ImageIO.read(new FileInputStream("/home/francescorusin/Downloads/Ruby.jpg"));
        final int width = ruby.getWidth();
        final int height = ruby.getHeight();
        final int[] rubyPixels = new int[width * height];
        ruby.getRGB(0, 0, width, height, rubyPixels, 0, width);
        ByteBuffer rubyBuffer = BufferUtils.createByteBuffer(width * height * 4);
        for (int i = 0; i < width; ++i) {
            for (int j = 0; j < height / 2; ++j) {
                final int pos1 = j * width + i;
                final int pos2 = (height - j - 1) * width + i;
                final int placeholder = rubyPixels[pos1];
                rubyPixels[pos1] = rubyPixels[pos2];
                rubyPixels[pos2] = placeholder;
            }
        }
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int pixel = rubyPixels[y * width + x];
                rubyBuffer.put((byte) ((pixel >> 16) & 0xFF));
                rubyBuffer.put((byte) ((pixel >> 8) & 0xFF));
                rubyBuffer.put((byte) (pixel & 0xFF));
                rubyBuffer.put((byte) ((pixel >> 24) & 0xFF));
            }
        }*/

        final int texture = glGenTextures();
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                BufferUtils.createByteBuffer(chessboard_pixels.length).put(chessboard_pixels).flip());

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

        final float[] vertexData =
                new float[]{
                        -.75f, -.75f, 0.0f, 0.0f,
                        -.75f, .75f, 0.0f, 1.0f,
                        .75f, -.75f, 1.0f, 0.0f,
                        .75f, .75f, 1.0f, 1.0f
                };
        final int[] indexData =
                new int[] {
                        0, 1, 2,
                        1, 2, 3
                };
        final int vertexBuffer = glGenBuffers();
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferData(GL_ARRAY_BUFFER, vertexData, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, false, 16, BufferUtils.createFloatBuffer(0));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, false, 16, BufferUtils.createFloatBuffer(0));

        while (!glfwWindowShouldClose(windowID)) {
            checkErrors();
            glClearColor(0f, 0f, 0f, 1f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glDrawArrays(GL_TRIANGLES, 0, 3);
            glDrawArrays(GL_TRIANGLES, 1, 4);

            glfwSwapBuffers(windowID);
            glfwPollEvents();
        }
        // glDeleteTextures(texture);
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
