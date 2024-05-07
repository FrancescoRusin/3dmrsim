package viewer;

import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.opengl.GL;

import java.util.Map;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.glfw.GLFW.glfwShowWindow;

public class ActualViewer /*implements Viewer*/ {
    private static final GLFWErrorCallback errorCallback = GLFWErrorCallback.createPrint(System.err);

    public ActualViewer() {
        glfwSetErrorCallback(errorCallback);
        if (!glfwInit()) {
            throw new IllegalStateException("Unable to initialize GLFW");
        }
        String title = "HAHA!";
        boolean resizable = true;
        int m_width = 1024;
        int m_height = 768;

        glfwDefaultWindowHints(); // Loads GLFW's default window settings
        glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE); // Sets window to be visible
        glfwWindowHint(GLFW_RESIZABLE, resizable ? GLFW_TRUE : GLFW_FALSE); // Sets whether the window is resizable

        long id = glfwCreateWindow(m_width, m_height, title, 0, 0); // Does the actual window creation

        glfwMakeContextCurrent(id); // glfwSwapInterval needs a context on the calling thread, otherwise will cause NO_CURRENT_CONTEXT error
        GL.createCapabilities(); // Will let lwjgl know we want to use this context as the context to draw with

        glfwSwapInterval(1); // How many draws to swap the buffer
        glfwShowWindow(id); // Shows the window
    }

    public static void main(String[] args) {
        Map<Integer, Integer> map = Map.of(1, 1, 2, 2, 3, 3);
        System.out.println(map.get(null));
        new ActualViewer();
    }
}
