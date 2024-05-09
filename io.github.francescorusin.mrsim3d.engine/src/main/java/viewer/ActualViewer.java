package viewer;

import geometry.Vector3D;
import org.lwjgl.BufferUtils;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.opengl.GL;

import java.nio.FloatBuffer;
import java.util.Arrays;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.glfw.GLFW.glfwShowWindow;
import static org.lwjgl.opengl.GL21.*;

public class ActualViewer /*implements Viewer*/ {
    private static final GLFWErrorCallback errorCallback = GLFWErrorCallback.createPrint(System.err);

    private static float[] rotate(float[] origin) {
        Vector3D rotation = new Vector3D(.01, 0d, 0d);
        Vector3D v1 = new Vector3D(origin[0], origin[1], origin[2]).rotate(rotation);
        Vector3D v2 = new Vector3D(origin[3], origin[4], origin[5]).rotate(rotation);
        Vector3D v3 = new Vector3D(origin[6], origin[7], origin[8]).rotate(rotation);
        return new float[]{
                (float) v1.x(), (float) v1.y(), (float) v1.z(),
                (float) v2.x(), (float) v2.y(), (float) v2.z(),
                (float) v3.x(), (float) v3.y(), (float) v3.z()
        };
    }

    private static float t = 0;

    private static float[] obscillate(float[] origin) {
        t += 1f / 60f;
        float[] result = Arrays.copyOf(origin, 9);
        float sin = (float) Math.sin(t);
        for (int i = 2; i < 9; i += 3) {
            result[i] *= sin;
        }
        return result;
    }

    public ActualViewer() {
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
        glClearColor(0f, 0f, 0f, 0f);
        glfwShowWindow(windowID); // Shows the window
        float[] origin = new float[]{
                0f, 1f / (float) Math.sqrt(3), 1f,
                -0.5f, -.5f / (float) Math.sqrt(3), 1f,
                0.5f, -.5f / (float) Math.sqrt(3), 1f
        };
        float[] obscillation;
        FloatBuffer vertexData = BufferUtils.createFloatBuffer(9);
        vertexData.put(origin);
        vertexData.flip();
        FloatBuffer triangleColorData = BufferUtils.createFloatBuffer(9);
        triangleColorData.put(new float[]{
                1f, 0f, 0f,
                0f, 1f, 0f,
                0f, 0f, 1f
        }).flip();
        int triangleVertexBuffer = glGenBuffers();
        glBindBuffer(GL_ARRAY_BUFFER, triangleVertexBuffer);
        glBufferData(GL_ARRAY_BUFFER, vertexData, GL_DYNAMIC_DRAW);
        int triangleColorBuffer = glGenBuffers();
        glBindBuffer(GL_ARRAY_BUFFER, triangleColorBuffer);
        glBufferData(GL_ARRAY_BUFFER, triangleColorData, GL_STATIC_DRAW);

        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);

        //TODO glFrustum, glLight, setCamera

        while ( !glfwWindowShouldClose(windowID) ) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear the framebuffer

            vertexData.clear();
            obscillation = obscillate(origin);
            vertexData.put(obscillation).flip();

            glBindBuffer(GL_ARRAY_BUFFER, triangleVertexBuffer);
            glBufferData(GL_ARRAY_BUFFER, vertexData, GL_STREAM_DRAW);
            glVertexPointer(3, GL_FLOAT, 0, 0);

            glBindBuffer(GL_ARRAY_BUFFER, triangleColorBuffer);
            glColorPointer(3, GL_FLOAT, 0, 0);

            glDrawArrays(GL_TRIANGLES, 0, 3);

            glDrawArrays(GL_TRIANGLES, 0, 3);

            glfwSwapBuffers(windowID); // swap the color buffers

            // Poll for window events. The key callback above will only be
            // invoked during this call.
            glfwPollEvents();
        }
    }

    public static void main(String[] args) {
        new ActualViewer();
    }
}
