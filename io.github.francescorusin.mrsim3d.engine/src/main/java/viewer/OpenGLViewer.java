package viewer;

import geometry.Vector3D;
import org.joml.Matrix4f;
import org.lwjgl.BufferUtils;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.opengl.GL;
import org.lwjgl.stb.STBImage;
import org.lwjgl.system.MemoryStack;

import java.awt.*;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.system.MemoryStack.stackPush;

public abstract class OpenGLViewer extends Viewer {
    protected static final Vector3D DEFAULT_CAMERA_POS = new Vector3D(5, -5, 5);
    protected static final Vector3D DEFAULT_CAMERA_DIR = new Vector3D(-5, 5, -3);
    protected static final Vector3D DEFAULT_CAMERA_UP = new Vector3D(0, 0, 1);
    protected Vector3D cameraPos;
    protected Vector3D cameraDir;
    protected final Vector3D cameraUp;

    public OpenGLViewer(Mode mode, Vector3D cameraPos, Vector3D cameraDir, Vector3D cameraUp) {
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
    }

    public OpenGLViewer(Mode mode) {
        this(mode, DEFAULT_CAMERA_POS, DEFAULT_CAMERA_DIR, DEFAULT_CAMERA_UP);
    }

    // This needs to be called after an OpenGL window is created to initialize stuff correctly
    protected void GLCapabilities() {
        GL.createCapabilities();
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_TEXTURE_2D);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    }

    protected void setView() {
        Vector3D cameraTarget = cameraPos.sum(cameraDir);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glFrustum(-0.5, 0.5, -0.5, 0.5, 0.5, 100);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glLoadMatrixf(
                new Matrix4f().lookAt(
                        (float) cameraPos.x(), (float) cameraPos.y(), (float) cameraPos.z(),
                        (float) cameraTarget.x(), (float) cameraTarget.y(), (float) cameraTarget.z(),
                        (float) cameraUp.x(), (float) cameraUp.y(), (float) cameraUp.z()
                ).get(BufferUtils.createFloatBuffer(16))
        );
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
}
