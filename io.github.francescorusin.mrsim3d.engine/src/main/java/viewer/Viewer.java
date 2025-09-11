/*-
 * ========================LICENSE_START=================================
 * mrsim3d.engine
 * %%
 * Copyright (C) 2024 - 2025 Francesco Rusin
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

import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.opengl.GL;

public class Viewer {
  private static final GLFWErrorCallback errorCallback = GLFWErrorCallback.createPrint(System.err);

  public Viewer() {
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
    glfwWindowHint(
        GLFW_RESIZABLE, resizable ? GLFW_TRUE : GLFW_FALSE); // Sets whether the window is resizable

    long id = glfwCreateWindow(m_width, m_height, title, 0, 0); // Does the actual window creation

    glfwMakeContextCurrent(
        id); // glfwSwapInterval needs a context on the calling thread, otherwise will cause
    // NO_CURRENT_CONTEXT error
    GL.createCapabilities(); // Will let lwjgl know we want to use this context as the context to
    // draw with

    glfwSwapInterval(1); // How many draws to swap the buffer
    glfwShowWindow(id); // Shows the window
  }

  public static void main(String[] args) {
    new Viewer();
  }
}
