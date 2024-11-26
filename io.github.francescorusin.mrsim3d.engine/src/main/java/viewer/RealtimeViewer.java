package viewer;

import geometry.Vector3D;
import org.lwjgl.BufferUtils;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.opengl.GL;
import outcome.InstantSnapshot;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Objects;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.glfw.GLFW.glfwShowWindow;
import static org.lwjgl.opengl.GL33.*;

public class RealtimeViewer implements Viewer {

    @Override
    public void initialize() {
        //TODO
    }

    @Override
    public void drawTriangle(Vector3D v1, Vector3D v2, Vector3D v3) {
        //TODO
    }

    @Override
    public void drawSphere(Vector3D position, double radius) {
        //TODO
    }

    @Override
    public void drawLine(Vector3D p1, Vector3D p2) {
        //TODO
    }

    @Override
    public void draw(InstantSnapshot instant) {
        //TODO
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

        glfwDefaultWindowHints(); // Loads GLFW's default window settings
        glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE); // Sets window to be visible
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE); // Sets whether the window is resizable

        long windowID = glfwCreateWindow(m_width, m_height, title, 0, 0); // Does the actual window creation

        // Setup a key callback. It will be called every time a key is pressed, repeated or released.
        glfwSetKeyCallback(windowID, (window, key, scancode, action, mods) -> {
            if ( key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE )
                glfwSetWindowShouldClose(window, true); // We will detect this in the rendering loop
        });

        glfwMakeContextCurrent(windowID); // glfwSwapInterval needs a context on the calling thread, otherwise will cause NO_CURRENT_CONTEXT error
        GL.createCapabilities(); // Will let lwjgl know we want to use this context as the context to draw with

        glfwSwapInterval(1); // How many draws to swap the buffer
        glClearColor(1f, 1f, 1f, 0f);
        glfwShowWindow(windowID); // Shows the window

        /*final float[] chessboard_pixels = new float[64 * 64 * 3];
        for (int i = 0; i < 64; ++i) {
            for (int j = 0; j < 64; ++j) {
                for (int k = 0; k < 3; ++k) {
                    chessboard_pixels[(i * 64 + j) * 3 + k] = ((i / 8 + j / 8) % 2) * .5f;
                }
            }
        }
        final int texture = glGenTextures();
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 64, 64, 0, GL_RGB, GL_FLOAT,
                BufferUtils.createFloatBuffer(64 * 64 * 3).put(chessboard_pixels));*/

        final int shader = createShader(extractShader("io.github.francescorusin.mrsim3d.engine/src/main/java/viewer/shader.txt"));
        glUseProgram(shader);

        final float[] vertices = new float[]{
            -.5f, -.5f,
            -.5f,  .5f,
             .5f, -.5f,
             .5f,  .5f
        };
        final int vertexBuffer = glGenBuffers();
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferData(GL_ARRAY_BUFFER, vertices, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, false, 8, BufferUtils.createFloatBuffer(8).put(vertices));

        while ( !glfwWindowShouldClose(windowID) ) {
            glClearColor(0f, 0f, 0f, 1f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

            glfwSwapBuffers(windowID);
            glfwPollEvents();
        }
        //glDeleteTextures(texture);
    }

    public static void main(String[] args) throws Exception {
        new RealtimeViewer();
    }
}
